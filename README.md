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

Here are the 24 Flashing Commands supported by the BL602 EFlash Loader, as decoded by Ghidra from `eflash_loader_cmds`...

| ID | ASCII | Flashing Command
| :--: | :--: | --- 
| 10 | LF | [bflb_eflash_loader_cmd_get_bootinfo](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L2867-L2879)
| 21 | ! | [bflb_eflash_loader_cmd_reset](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L2939-L2950)
| 30 | 0 | [bflb_eflash_loader_cmd_erase_flash](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3133-L3194)
| 31 | 1 | [bflb_eflash_loader_cmd_write_flash](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3258-L3300)
| 3f | ? | [bflb_eflash_loader_cmd_write_flash_with_decompress](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3693-L3798)
| 32 | 2 | [bflb_eflash_loader_cmd_read_flash](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3374-L3427)
| 34 | 4 | [bflb_eflash_loader_cmd_xip_read_flash](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3434-L3487)
| 3a | : | [bflb_eflash_loader_cmd_write_flash_check](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3001-L3008)
| 3b | ; | [bflb_eflash_loader_cmd_set_flash_para](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3635-L3689)
| 3c | < | [bflb_eflash_loader_cmd_flash_chip_erase](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3113-L3129)
| 3d | = | [bflb_eflash_loader_cmd_readSha_flash](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3491-L3544)
| 3e | > | [bflb_eflash_loader_cmd_xip_readSha_flash](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3548-L3601)
| 40 | @ | [bflb_eflash_loader_cmd_write_efuse](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3065-L3109)
| 41 | A | [bflb_eflash_loader_cmd_read_efuse](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3014-L3058)
| 42 | B | [bflb_eflash_loader_cmd_read_mac_addr](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3605-L3629)
| 50 | P | [bflb_eflash_loader_cmd_write_mem](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L2975-L2997)
| 51 | Q | [bflb_eflash_loader_cmd_read_mem](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3213-L3254)
| 71 | q | [bflb_eflash_loader_cmd_read_log](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L2897-L2909)
| 60 | ` | [bflb_eflash_loader_cmd_xip_read_flash_start](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L2913-L2922)
| 61 | a | [bflb_eflash_loader_cmd_xip_read_flash_finish](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L2926-L2935)
| 36 | 6 | [bflb_eflash_loader_cmd_read_jedec_id](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L2883-L2893)
| 37 | 7 | [bflb_eflash_loader_cmd_read_status_register](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3339-L3367)
| 38 | 8 | [bflb_eflash_loader_cmd_write_status_register](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3306-L3335)
| 33 | 3 | [bflb_eflash_loader_cmd_flash_boot](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3198-L3209)

# Flashing States

BL602 Firmware Flasher works like a State Machine. Each Flashing State triggers a Flashing Command. Here are the Flashing States defined in [`BLOpenFlasher/utils/util_program.go`](https://github.com/bouffalolab/BLOpenFlasher/blob/main/utils/util_program.go)...

| State | ID | On Success | On Error |
| :--- | :--- | :--- | :--- |
| [ConfigReset](https://github.com/bouffalolab/BLOpenFlasher/blob/main/utils/util_program.go#L118-L133) | | CmdReset | ErrorLoaderBin
| [CmdReset](https://github.com/bouffalolab/BLOpenFlasher/blob/main/utils/util_program.go#L135-L193) | | CmdShakeHand | ErrorShakeHand
| [CmdShakeHand](https://github.com/bouffalolab/BLOpenFlasher/blob/main/utils/util_program.go#L195-L206) | 55 | CmdBootInfo | CmdReset
| [CmdBootInfo](https://github.com/bouffalolab/BLOpenFlasher/blob/main/utils/util_program.go#L208-L215) | 10 | CmdBootHeader | CmdReset
| [CmdBootHeader](https://github.com/bouffalolab/BLOpenFlasher/blob/main/utils/util_program.go#L217-L230) | 11 | CmdSegHeader | ConfigReset
| [CmdSegHeader](https://github.com/bouffalolab/BLOpenFlasher/blob/main/utils/util_program.go#L232-L245) | 17 | CmdSegData | ConfigReset
| [CmdSegData](https://github.com/bouffalolab/BLOpenFlasher/blob/main/utils/util_program.go#L247-L264) | 18 | CmdCheckImage | ConfigReset
| [CmdCheckImage](https://github.com/bouffalolab/BLOpenFlasher/blob/main/utils/util_program.go#L266-L274) | 19 | CmdRunImage | ConfigReset
| [CmdRunImage](https://github.com/bouffalolab/BLOpenFlasher/blob/main/utils/util_program.go#L276-L284) | 1a | CmdReshake | ConfigReset
| [CmdReshake](https://github.com/bouffalolab/BLOpenFlasher/blob/main/utils/util_program.go#L286-L300) | 55 | CmdLoadFile | ConfigReset
| [CmdLoadFile](https://github.com/bouffalolab/BLOpenFlasher/blob/main/utils/util_program.go#L302-L344) |  | CmdEraseFlash^ | ErrorOpenFile^
| [CmdEraseFlash](https://github.com/bouffalolab/BLOpenFlasher/blob/main/utils/util_program.go#L353-L378) | 30 | CmdProgramFlash | ErrorEraseFlash
| [CmdProgramFlash](https://github.com/bouffalolab/BLOpenFlasher/blob/main/utils/util_program.go#L380-L408) | 31 | CmdProgramOK^ | ErrorProgramFLash
| [CmdProgramOK](https://github.com/bouffalolab/BLOpenFlasher/blob/main/utils/util_program.go#L410-L418) | 3a | CmdSha256 | ErrorProgramOK
| [CmdSha256](https://github.com/bouffalolab/BLOpenFlasher/blob/main/utils/util_program.go#L420-L449) | 3d | CmdLoadFile | ErrorVerifySha256^
| [CmdProgramFinish](https://github.com/bouffalolab/BLOpenFlasher/blob/main/utils/util_program.go#L451-L468) | 55 | CmdProgramFinish | CmdProgramFinish

^ denotes multiple states
