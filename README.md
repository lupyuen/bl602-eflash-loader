# BL602 EFlash Loader decompiled with Ghidra

Read the article...

-   ["BL602 EFlash Loader: Reverse Engineered with Ghidra"](https://lupyuen.github.io/articles/loader)

BL602 EFlash Loader is the program that runs on BL602 to flash all firmware to its Embedded Flash. The ELF was uploaded recently (no source available, according to GitHub Code Search).

To understand what's inside BL602 EFlash Loader, we decompiled with [Ghidra](https://ghidra-sre.org/) this official ELF from BL IoT SDK...

-   [bl_iot_sdk/flash_tool/chips/bl602/eflash_loader/eflash_loader.elf](https://github.com/bouffalolab/bl_iot_sdk/blob/master/flash_tool/chips/bl602/eflash_loader/eflash_loader.elf)

    (Dated 17 Jan 2022)

Below is the decompiled C source file...

-   [eflash_loader.c](eflash_loader.c)

More about BL602 EFlash Loader...

-   ["Flashing Firmware to PineCone BL602"](https://lupyuen.github.io/articles/flash)

More about Ghidra...

-   ["Ghidra: Software reverse engineering suite of tools"](https://ghidra-sre.org/)

# Flashing Commands

Here are the 24 Flashing Commands supported by the BL602 EFlash Loader, as decoded by Ghidra from `eflash_loader_cmds`...

| ID | ASCII | Flashing Command
| :--: | :--: | --- 
| 10 | LF | [bflb_eflash_loader_cmd_get_bootinfo](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L2867-L2879)
| 21 | ! | [bflb_eflash_loader_cmd_reset](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L2939-L2950)
| 30 | 0 | [bflb_eflash_loader_cmd_erase_flash](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3133-L3194)
| 31 | 1 | [bflb_eflash_loader_cmd_write_flash](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3258-L3300)
| 3F | ? | [bflb_eflash_loader_cmd_write_flash_with_decompress](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3693-L3798)
| 32 | 2 | [bflb_eflash_loader_cmd_read_flash](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3374-L3427)
| 34 | 4 | [bflb_eflash_loader_cmd_xip_read_flash](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3434-L3487)
| 3A | : | [bflb_eflash_loader_cmd_write_flash_check](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3001-L3008)
| 3B | ; | [bflb_eflash_loader_cmd_set_flash_para](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3635-L3689)
| 3C | < | [bflb_eflash_loader_cmd_flash_chip_erase](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3113-L3129)
| 3D | = | [bflb_eflash_loader_cmd_readSha_flash](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3491-L3544)
| 3E | > | [bflb_eflash_loader_cmd_xip_readSha_flash](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3548-L3601)
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

7 of the above Flashing Commands are documented in the [BL602 ISP Protocol](https://github.com/bouffalolab/bl_docs/tree/main/BL602_ISP/en)...

-   `10` - Get Boot Info
-   `3C` - Chip Erase
-   `30` - Flash Erase
-   `31` - Flash Program
-   `3A` - Flash Program Check
-   `32` - Flash Read
-   `3D` - SHA256 Read

The other 17 Flashing Commands are undocumented.

# Flashing States

BL602 Firmware Flasher works like a State Machine. Each Flashing State triggers a Flashing Command. Here are the Flashing States and Flashing Command IDs derived from [`BLOpenFlasher/utils/util_program.go`](https://github.com/bouffalolab/BLOpenFlasher/blob/main/utils/util_program.go)...

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
| [CmdRunImage](https://github.com/bouffalolab/BLOpenFlasher/blob/main/utils/util_program.go#L276-L284) | 1A | CmdReshake | ConfigReset
| [CmdReshake](https://github.com/bouffalolab/BLOpenFlasher/blob/main/utils/util_program.go#L286-L300) | 55 | CmdLoadFile | ConfigReset
| [CmdLoadFile](https://github.com/bouffalolab/BLOpenFlasher/blob/main/utils/util_program.go#L302-L344) |  | CmdEraseFlash^ | ErrorOpenFile^
| [CmdEraseFlash](https://github.com/bouffalolab/BLOpenFlasher/blob/main/utils/util_program.go#L353-L378) | 30 | CmdProgramFlash | ErrorEraseFlash
| [CmdProgramFlash](https://github.com/bouffalolab/BLOpenFlasher/blob/main/utils/util_program.go#L380-L408) | 31 | CmdProgramOK^ | ErrorProgramFLash
| [CmdProgramOK](https://github.com/bouffalolab/BLOpenFlasher/blob/main/utils/util_program.go#L410-L418) | 3A | CmdSha256 | ErrorProgramOK
| [CmdSha256](https://github.com/bouffalolab/BLOpenFlasher/blob/main/utils/util_program.go#L420-L449) | 3D | CmdLoadFile | ErrorVerifySha256^
| [CmdProgramFinish](https://github.com/bouffalolab/BLOpenFlasher/blob/main/utils/util_program.go#L451-L468) | 55 | CmdProgramFinish | CmdProgramFinish

^ denotes multiple states

The Flashing Process is documented in the [BL602 ISP Protocol](https://github.com/bouffalolab/bl_docs/tree/main/BL602_ISP/en).

# Matching Flashing States and Commands

By matching the Flashing States and the Flashing Commands above, we identify 5 commands that we can probe further...

| ID | ASCII | Flashing Command
| :--: | :--: | --- 
| 10 | LF | Get Boot Info<br>[bflb_eflash_loader_cmd_get_bootinfo](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L2867-L2879)
| 30 | 0 | Flash Erase<br>[bflb_eflash_loader_cmd_erase_flash](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3133-L3194)
| 31 | 1 | Flash Program<br>[bflb_eflash_loader_cmd_write_flash](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3258-L3300)
| 3A | : | Flash Program Check<br>[bflb_eflash_loader_cmd_write_flash_check](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3001-L3008)
| 3D | = | SHA256 Read<br>[bflb_eflash_loader_cmd_readSha_flash](https://github.com/lupyuen/bl602-eflash-loader/blob/main/eflash_loader.c#L3491-L3544)

(`3C` Chip Erase and `32` Flash Read aren't used while flashing BL602, according to BLOpenFlasher)
