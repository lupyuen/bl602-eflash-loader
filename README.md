# BL602 EFlash Loader decompiled with Ghidra

Follow the Reverse Engineering updates on Twitter...

https://twitter.com/MisterTechBlog/status/1486187004232867842

BL602 EFlash Loader decompiled with Ghidra from...

https://github.com/bouffalolab/bl_iot_sdk/blob/master/flash_tool/chips/bl602/eflash_loader/eflash_loader.elf

(Dated 17 Jan 2022)

Decompiled C source file...

[`eflash_loader.c`](eflash_loader.c)

More about BL602 EFlash Loader...

https://lupyuen.github.io/articles/flash

# Flashing Commands

Here are the Flashing Commands supported by the BL602 EFlash Loader, as decoded by Ghidra from `eflash_loader_cmds`...

| ID | ASCII | Flashing Command
| :--: | :--: | --- 
| 10 | LF | bflb_eflash_loader_cmd_get_bootinfo
| 21 | ! | bflb_eflash_loader_cmd_reset
| 30 | 0 | bflb_eflash_loader_cmd_erase_flash
| 31 | 1 | bflb_eflash_loader_cmd_write_flash
| 3f | ? | bflb_eflash_loader_cmd_write_flash_with_decompress
| 32 | 2 | bflb_eflash_loader_cmd_read_flash
| 34 | 4 | bflb_eflash_loader_cmd_xip_read_flash
| 3a | : | bflb_eflash_loader_cmd_write_flash_check
| 3b | ; | bflb_eflash_loader_cmd_set_flash_para
| 3c | < | bflb_eflash_loader_cmd_flash_chip_erase
| 3d | = | bflb_eflash_loader_cmd_readSha_flash
| 3e | > | bflb_eflash_loader_cmd_xip_readSha_flash
| 40 | @ | bflb_eflash_loader_cmd_write_efuse
| 41 | A | bflb_eflash_loader_cmd_read_efuse
| 42 | B | bflb_eflash_loader_cmd_read_mac_addr
| 50 | P | bflb_eflash_loader_cmd_write_mem
| 51 | Q | bflb_eflash_loader_cmd_read_mem
| 71 | q | bflb_eflash_loader_cmd_read_log
| 60 | ` | bflb_eflash_loader_cmd_xip_read_flash_start
| 61 | a | bflb_eflash_loader_cmd_xip_read_flash_finish
| 36 | 6 | bflb_eflash_loader_cmd_read_jedec_id
| 37 | 7 | bflb_eflash_loader_cmd_read_status_register
| 38 | 8 | bflb_eflash_loader_cmd_write_status_register
| 33 | 3 | bflb_eflash_loader_cmd_flash_boot
