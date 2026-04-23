from intelhex import IntelHex

# 加载 Bootloader 和 App 的 HEX 文件
ih_boot = IntelHex()
ih_boot.loadhex('BIAI_UFO_BOOT_V1.1.hex')

ih_app = IntelHex()
ih_app.loadhex('output\BIAI_UFO_APP.hex')

# 将 App 合并到 Bootloader 上（地址重叠时 App 覆盖 Bootloader）
ih_boot.merge(ih_app, overlap='replace')

# 保存合并后的 HEX 文件
ih_boot.write_hex_file('BIAI_UFO_ALL.hex')

