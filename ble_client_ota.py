import asyncio
from bleak import BleakClient, BleakScanner  # 导入BleakScanner
import binascii
import time
import os
import struct
import math


### 待填部分

# 连上直连设备后，按提示按1回车

DEVICE_ADDRESS = '83:29:7C:B8:F4:BE'    # glass sample 1

FIRMWARE_FILE = 'BIAI_UFO_APP_OTA.bin'  # 更改为你的固件文件路径

VER_H = 1   # V1.2 版本
VER_L = 2

OUTPUT_FILE = 'notify_data.txt'

#########################################################


notify_pk_count = 0
write_characteristic_uuid = None

### ota升级相关代码
class OTAState:
    IDLE = 0
    STARTED = 1
    SENDING = 2
    VERIFYING = 3
    COMPLETED = 4


# 命令定义
OTA_CMD_START = 0xD0     
OTA_CMD = 0xD1  
OTA_BIN_INFO_SUBCMD = 0x01  
OTA_BIN_INFO_DEV_RESP_SUBCMD = 0x02  

OTA_DATA_SUBCMD = 0x0f 
OTA_DATA_DEV_RESP_SUBCMD = 0x03  

OTA_FINISH_VERIFY_SUBCMD = 0x04      
OTA_DEV_VERIFY_RESULT_SUBCMD = 0x05    
    
# OTA状态
OTA_RESULT_SUCCESS = 0x00
OTA_RESULT_FAIL = 0x01

# 全局变量
ota_in_progress = OTAState.IDLE
firmware_data = None
firmware_size = 0
fw_version = VER_H << 8 | VER_L
block_size = 128  
current_block = 0
total_blocks = 0
loop_blocks_num = 10   
crc16_value = 0



def crc16_modbus(init_value, ptr, length):
    # crc = 0xFFFF  #第一次的值
    crc = init_value
    for _ in range(length):
        crc ^= ptr[_]
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc
# 特征值属性解析函数
def parse_characteristic_properties(properties):
    """解析特征值属性并返回可读字符串"""
    prop_map = {
        'broadcast': '广播',
        'read': '读取',
        'write-without-response': '无响应写入',
        'write': '写入',
        'notify': '通知',
        'indicate': '指示',
        'signed-write': '签名写入',
        'extended-properties': '扩展属性'
    }
    return [prop_map.get(p, p) for p in properties]


# ：BLE设备发现回调函数
def discovery_callback(device, advertisement_data):
    """处理发现的BLE设备"""
    print("\n" + "="*50)
    print(f"发现设备: {device.name} ({device.address})")
    print(f"信号强度: {device.rssi} dBm")
    
    # 打印广播数据
    if advertisement_data.service_uuids:
        print("服务UUIDs:")
        for uuid in advertisement_data.service_uuids:
            print(f"  - {uuid}")
    
    if advertisement_data.service_data:
        print("服务数据:")
        for uuid, data in advertisement_data.service_data.items():
            hex_data = binascii.hexlify(data).decode('utf-8')
            print(f"  - {uuid}: {hex_data}")
    
    if advertisement_data.manufacturer_data:
        print("制造商数据:")
        for m_id, data in advertisement_data.manufacturer_data.items():
            hex_data = binascii.hexlify(data).decode('utf-8')
            print(f"  - 制造商ID: 0x{m_id:04X}, 数据: {hex_data}")
    
    if advertisement_data.local_name:
        print(f"本地名称: {advertisement_data.local_name}")
    
    if advertisement_data.tx_power is not None:
        print(f"发射功率: {advertisement_data.tx_power} dBm")

async def scan_ble_devices():
    
    # 创建扫描器并设置回调
    scanner = BleakScanner()
    scanner.register_detection_callback(discovery_callback)
    
    try:
        # 开始扫描（持续3秒）
        await scanner.start()
        await asyncio.sleep(3)
        await scanner.stop()
        
        # 获取并打印所有发现的设备
        devices = await scanner.get_discovered_devices()
        print("\n扫描结束，发现设备列表:")
        for i, device in enumerate(devices, 1):
            print(f"{i}. {device.name} ({device.address}) - RSSI: {device.rssi} dBm")
        
        return devices
    except asyncio.CancelledError:
        await scanner.stop()
        print("\n扫描已取消")

async def notification_handler(sender, data):
    global notify_pk_count, loop_blocks_num
    global ota_in_progress, current_block 
    # 将接收到的数据转换为十六进制字符串
    hex_data = binascii.hexlify(data).decode('utf-8')
    print(f"接收到数据包 {notify_pk_count + 1}: {hex_data}")
    # 打开文件以追加模式写入数据
    with open(OUTPUT_FILE, 'a') as file:
        file.write(f"{hex_data}\n")
    notify_pk_count += 1

    
    if ota_in_progress:
        command = data[0]
        if len(data) < 2:  
            sub_command = 0 
        else:
            sub_command = data[1]
        
        if command == OTA_CMD_START:
            pass
                
        elif command == OTA_CMD:
            if sub_command == OTA_DATA_DEV_RESP_SUBCMD:
                
                current_block = struct.unpack('<H', data[2:4])[0]    
                
                current_rx_bytes = struct.unpack('<I', data[4:8])[0]
                
                ota_in_progress = OTAState.SENDING
                current_block += 1
                await send_next_firmware_block()
                current_block += 1
            elif sub_command == OTA_BIN_INFO_DEV_RESP_SUBCMD:
                
                if data[2] == 1:
                    loop_blocks_num = data[3]
                    current_block = 0
                    await send_next_firmware_block()
                    current_block += 1
                    ota_in_progress = OTAState.SENDING
                else:
                    ota_in_progress = OTAState.IDLE
                
            elif sub_command == OTA_DEV_VERIFY_RESULT_SUBCMD:
                if data[2] == 1:
                    print("固件校验升级成功.")
                    ota_in_progress = OTAState.COMPLETED
                else:
                    print("固件校验失败，请重新传输")
                    ota_in_progress = OTAState.IDLE
                        
            else:
                print(f"sub cmd: {sub_command} 无法解析")
                
                

async def send_data_to_device(client, data):
    """向设备发送数据"""
    global write_characteristic_uuid
    
    if not write_characteristic_uuid:
        return False
    
    try:
        # 将数据转换为字节格式
        if isinstance(data, str):
            data_bytes = data.encode('utf-8')
        else:
            data_bytes = data
        
        # 发送数据
        await client.write_gatt_char(write_characteristic_uuid, data_bytes)
        print(f"成功发送数据: {binascii.hexlify(data_bytes).decode('utf-8')}")
        return True
    except Exception as e:
        print(f"发送数据失败: {e}")
        return False



async def start_ota_update(client):

    global ota_in_progress, firmware_data, firmware_size, block_size, current_block, total_blocks, crc16_value, fw_version
    
   
    if firmware_data is None:
        return False
    
    with open(FIRMWARE_FILE, 'rb') as file:
        crc_value = 0xFFFF
        for chunk in iter(lambda: file.read(4096), b''): 
            crc_value = crc16_modbus(crc_value, chunk, len(chunk))
        print(f"crc_value: {crc_value} ")
            
    
    # 计算总块数
    total_blocks = math.ceil(firmware_size / block_size)
    current_block = 0
    
    print(f"开始OTA更新, 固件大小: {firmware_size}字节, CRC16: {hex(crc16_value)}, 总块数: {total_blocks}")
    
    ota_in_progress = OTAState.STARTED
    command_data = struct.pack('<B', OTA_CMD_START)
    await send_data_to_device(client, command_data)
    
    n = 3
    while( n > 0):
        await asyncio.sleep(1)
        command_data = struct.pack('<BBBHIH', OTA_CMD, OTA_BIN_INFO_SUBCMD, 0, fw_version, firmware_size, crc_value)
        await send_data_to_device(client, command_data)
        await asyncio.sleep(1)
        if ota_in_progress != OTAState.STARTED:
            break
        n -= 1
    
    return True

async def send_next_firmware_block():
    global current_block
    
    if current_block < total_blocks:
        await send_firmware_block(current_block)
    else:
        await verify_firmware_Request()

async def send_firmware_block(block_index):
    global firmware_data, block_size, firmware_size
    
    # 计算数据块位置
    start = block_index * block_size
    end = min(start + block_size, firmware_size)
    block_data = firmware_data[start:end]
    
    if len(block_data) < block_size:
        block_data += b'\xFF' * (block_size - len(block_data))
    
    checksum = sum(block_data) & 0xFF 
    
    header = struct.pack('<BBHBB', OTA_CMD, OTA_DATA_SUBCMD, block_index, checksum, block_size)
    packet = header + block_data
    
    await send_data_to_device(client, packet)
    print(f"发送固件块 {block_index+1}/{total_blocks} ({len(block_data)}字节)")

async def verify_firmware_Request():
    global ota_in_progress
    ota_in_progress = OTAState.VERIFYING
    command_data = struct.pack('<BB', OTA_CMD, OTA_FINISH_VERIFY_SUBCMD)
    await send_data_to_device(client, command_data)


        
async def main():
    global write_characteristic_uuid, notify_characteristic_uuid, client, firmware_data, firmware_size, block_size
    global ota_in_progress, current_block
    try:
        discovered_devices = await scan_ble_devices()
        
        # 检查目标设备是否被发现
        target_device = next((d for d in discovered_devices if d.address.upper() == DEVICE_ADDRESS.upper()), None)
        
        if not target_device:
            print(f"\n警告：未找到目标设备 {DEVICE_ADDRESS}，尝试直接连接...")
        
        
        print(f"\n尝试连接到设备 {DEVICE_ADDRESS}...")
        async with BleakClient(DEVICE_ADDRESS) as client:
            if client.is_connected:
                print(f"成功连接到设备 {DEVICE_ADDRESS}。")
                
                current_mtu = client.mtu_size
                
                # 设置块大小为MTU - 头部大小
                block_size = current_mtu - 10  # 留出协议头部空间
                if block_size > 240:    # 协议限制为一个字节
                    block_size = 240
                    
                print(f"设置OTA块大小: {block_size}字节")
                
                # 读取固件文件
                try:
                    with open(FIRMWARE_FILE, 'rb') as f:
                        firmware_data = f.read()
                    firmware_size = len(firmware_data)
                    print(f"固件加载成功: {firmware_size}字节")
                except Exception as e:
                    print(f"加载固件失败: {e}")
                    firmware_data = None
                    
                # 发现设备的所有服务和特征值
                services = await client.get_services()
                notify_characteristics = []
                write_characteristics = []  # ：存储写入特征值
                
                # 遍历所有服务和特征值，找到支持通知的特征值
                for service in services:
                    for characteristic in service.characteristics:
                        # 解析属性
                        props = parse_characteristic_properties(characteristic.properties)
                        char_info = {
                            "service_uuid": service.uuid,
                            "char_uuid": characteristic.uuid,
                            "properties": props,
                            "descriptors": [d.uuid for d in characteristic.descriptors]
                        }
                        
                        if 'notify' in characteristic.properties:
                            notify_characteristics.append(characteristic.uuid)
                            print(f"发现支持通知的特征值: {characteristic.uuid}")
                        # ：查找支持写入的特征值
                        if 'write' in characteristic.properties or 'write-without-response' in characteristic.properties:
                            write_characteristics.append(characteristic.uuid)
                            print(f"发现支持写入的特征值: {characteristic.uuid}")
                            
                
                 # 检查特征值
                if not notify_characteristics:
                    print("警告: 未找到支持通知的特征值。")
                
                if not write_characteristics:
                    print("错误: 未找到支持写入的特征值，无法发送数据")
                else:
                    # 选择第一个写入特征值
                    write_characteristic_uuid = write_characteristics[0]
                    print(f"将使用写入特征值: {write_characteristic_uuid}")

                # 订阅通知特征值（如果有）
                if notify_characteristics:
                    target_uuid = notify_characteristics[0]
                    await client.start_notify(target_uuid, notification_handler)
                    print(f"已开始监听特征值 {target_uuid} 的通知。")

                
                choice = input("输入选项 (1-3): ").strip()
                print(f"您选择了选项 {choice}")
                if choice == '1' and firmware_data:
                    # 启动OTA更新
                    current_block_tmp = current_block
                    await start_ota_update(client)
                    
                    # 等待OTA完成
                    while ota_in_progress:
                        if ota_in_progress == OTAState.SENDING:
                            if (current_block % loop_blocks_num) != 0:
                                await send_next_firmware_block()
                                current_block += 1
                                
                            if current_block_tmp != current_block:
                                current_block_tmp = current_block
                                print("Sent:", current_block)
                            await asyncio.sleep(0.1)                           
                        else:
                            await asyncio.sleep(0.1)
                    print("OTA更新流程完成")
                elif choice == '2' and write_characteristic_uuid:
                    # 发送测试数据
                    print("\n发送测试命令...")
                    await send_data_to_device(client, "TEST_CMD")
                    
                    print("\n发送二进制数据...")
                    binary_data = bytes([0xF0, 0x02])  # INTO APP
                    await send_data_to_device(client, binary_data)
                    
                elif choice == '3' and notify_characteristics:
                    print("\n仅监听设备通知...")
                else:
                    print("无效选择或缺少必要条件")
                
                # 持续监听一段时间（如果启用了通知）
                if notify_characteristics:
                    print("\n持续监听设备通知...")
                    await asyncio.sleep(10 * 60)  # 监听10分钟
                    
                    # 停止通知
                    await client.stop_notify(notify_characteristic_uuid)
                    print("已停止监听通知。")
            else:
                print(f"无法连接到设备 {DEVICE_ADDRESS}。")
    except Exception as e:
        print(f"发生错误: {e}")
        ota_in_progress = OTAState.IDLE

if __name__ == "__main__":
    asyncio.run(main())