typedef unsigned char   undefined;

typedef unsigned char    bool;
typedef unsigned char    byte;
typedef unsigned int    dword;
typedef long long    longlong;
typedef unsigned long long    qword;
typedef unsigned char    uchar;
typedef unsigned int    uint;
typedef unsigned long    ulong;
typedef unsigned long long    ulonglong;
typedef unsigned char    undefined1;
typedef unsigned short    undefined2;
typedef unsigned int    undefined4;
typedef unsigned long long    undefined8;
typedef unsigned short    ushort;
typedef unsigned short    word;
typedef ulong size_t;

typedef bool _Bool;

typedef undefined complex long double[32];

typedef qword complex float;

typedef undefined complex double[16];

typedef struct SPI_Flash_Cfg_Type SPI_Flash_Cfg_Type, *PSPI_Flash_Cfg_Type;

typedef uchar uint8_t;

typedef ushort uint16_t;

struct SPI_Flash_Cfg_Type {
    uint8_t ioMode;
    uint8_t cReadSupport;
    uint8_t clkDelay;
    uint8_t clkInvert;
    uint8_t resetEnCmd;
    uint8_t resetCmd;
    uint8_t resetCreadCmd;
    uint8_t resetCreadCmdSize;
    uint8_t jedecIdCmd;
    uint8_t jedecIdCmdDmyClk;
    uint8_t qpiJedecIdCmd;
    uint8_t qpiJedecIdCmdDmyClk;
    uint8_t sectorSize;
    uint8_t mid;
    uint16_t pageSize;
    uint8_t chipEraseCmd;
    uint8_t sectorEraseCmd;
    uint8_t blk32EraseCmd;
    uint8_t blk64EraseCmd;
    uint8_t writeEnableCmd;
    uint8_t pageProgramCmd;
    uint8_t qpageProgramCmd;
    uint8_t qppAddrMode;
    uint8_t fastReadCmd;
    uint8_t frDmyClk;
    uint8_t qpiFastReadCmd;
    uint8_t qpiFrDmyClk;
    uint8_t fastReadDoCmd;
    uint8_t frDoDmyClk;
    uint8_t fastReadDioCmd;
    uint8_t frDioDmyClk;
    uint8_t fastReadQoCmd;
    uint8_t frQoDmyClk;
    uint8_t fastReadQioCmd;
    uint8_t frQioDmyClk;
    uint8_t qpiFastReadQioCmd;
    uint8_t qpiFrQioDmyClk;
    uint8_t qpiPageProgramCmd;
    uint8_t writeVregEnableCmd;
    uint8_t wrEnableIndex;
    uint8_t qeIndex;
    uint8_t busyIndex;
    uint8_t wrEnableBit;
    uint8_t qeBit;
    uint8_t busyBit;
    uint8_t wrEnableWriteRegLen;
    uint8_t wrEnableReadRegLen;
    uint8_t qeWriteRegLen;
    uint8_t qeReadRegLen;
    uint8_t releasePowerDown;
    uint8_t busyReadRegLen;
    uint8_t readRegCmd[4];
    uint8_t writeRegCmd[4];
    uint8_t enterQpi;
    uint8_t exitQpi;
    uint8_t cReadMode;
    uint8_t cRExit;
    uint8_t burstWrapCmd;
    uint8_t burstWrapCmdDmyClk;
    uint8_t burstWrapDataMode;
    uint8_t burstWrapData;
    uint8_t deBurstWrapCmd;
    uint8_t deBurstWrapCmdDmyClk;
    uint8_t deBurstWrapDataMode;
    uint8_t deBurstWrapData;
    uint16_t timeEsector;
    uint16_t timeE32k;
    uint16_t timeE64k;
    uint16_t timePagePgm;
    uint16_t timeCe;
    uint8_t pdDelay;
    uint8_t qeData;
};

typedef struct UART_CFG_Type UART_CFG_Type, *PUART_CFG_Type;

typedef uint uint32_t;

typedef enum UART_DataBits_Type {
    UART_DATABITS_5=0,
    UART_DATABITS_6=1,
    UART_DATABITS_7=2,
    UART_DATABITS_8=3
} UART_DataBits_Type;

typedef enum UART_StopBits_Type {
    UART_STOPBITS_1=0,
    UART_STOPBITS_1_5=1,
    UART_STOPBITS_2=2
} UART_StopBits_Type;

typedef enum UART_Parity_Type {
    UART_PARITY_NONE=0,
    UART_PARITY_ODD=1,
    UART_PARITY_EVEN=2
} UART_Parity_Type;

typedef enum BL_Fun_Type {
    DISABLE=0,
    ENABLE=1
} BL_Fun_Type;

typedef enum UART_ByteBitInverse_Type {
    UART_LSB_FIRST=0,
    UART_MSB_FIRST=1
} UART_ByteBitInverse_Type;

struct UART_CFG_Type {
    uint32_t uartClk;
    uint32_t baudRate;
    enum UART_DataBits_Type dataBits;
    enum UART_StopBits_Type stopBits;
    enum UART_Parity_Type parity;
    enum BL_Fun_Type ctsFlowControl;
    enum BL_Fun_Type rxDeglitch;
    enum BL_Fun_Type rtsSoftwareControl;
    enum UART_ByteBitInverse_Type byteBitInverse;
    undefined field9_0xf;
};

typedef struct UART_FifoCfg_Type UART_FifoCfg_Type, *PUART_FifoCfg_Type;

struct UART_FifoCfg_Type {
    uint8_t txFifoDmaThreshold;
    uint8_t rxFifoDmaThreshold;
    enum BL_Fun_Type txFifoDmaEnable;
    enum BL_Fun_Type rxFifoDmaEnable;
};

typedef enum UART_ID_Type {
    UART0_ID=0,
    UART1_ID=1,
    UART_ID_MAX=2
} UART_ID_Type;

typedef enum UART_AutoBaudDetection_Type {
    UART_AUTOBAUD_0X55=0,
    UART_AUTOBAUD_STARTBIT=1
} UART_AutoBaudDetection_Type;

typedef enum UART_Direction_Type {
    UART_TX=0,
    UART_RX=1,
    UART_TXRX=2
} UART_Direction_Type;

typedef enum UART_Overflow_Type {
    UART_TX_OVERFLOW=0,
    UART_TX_UNDERFLOW=1,
    UART_RX_OVERFLOW=2,
    UART_RX_UNDERFLOW=3
} UART_Overflow_Type;

typedef struct UART_IrCfg_Type UART_IrCfg_Type, *PUART_IrCfg_Type;

struct UART_IrCfg_Type {
    enum BL_Fun_Type txIrEnable;
    enum BL_Fun_Type rxIrEnable;
    enum BL_Fun_Type txIrInverse;
    enum BL_Fun_Type rxIrInverse;
    uint16_t txIrPulseStart;
    uint16_t txIrPulseStop;
    uint16_t rxIrPulseStart;
};

typedef enum UART_INT_Type {
    UART_INT_TX_END=0,
    UART_INT_RX_END=1,
    UART_INT_TX_FIFO_REQ=2,
    UART_INT_RX_FIFO_REQ=3,
    UART_INT_RTO=4,
    UART_INT_PCE=5,
    UART_INT_TX_FER=6,
    UART_INT_RX_FER=7,
    UART_INT_ALL=8
} UART_INT_Type;

typedef enum flag {
    FL_UPPER=-128,
    FL_ZERO=1,
    FL_MINUS=2,
    FL_PLUS=4,
    FL_TICK=8,
    FL_SPACE=16,
    FL_HASH=32,
    FL_SIGNED=64
} flag;

typedef enum ranks {
    rank_char=-2,
    rank_short=-1,
    rank_int=0,
    rank_long=1,
    rank_longlong=2
} ranks;

typedef enum anon_enum_8.conflictaf6e {
    st_normal=0,
    st_flags=1,
    st_width=2,
    st_prec=3,
    st_modifiers=4
} anon_enum_8.conflictaf6e;

typedef void * __gnuc_va_list;

typedef __gnuc_va_list va_list;

typedef struct pHidSdio_RegMap_t pHidSdio_RegMap_t, *PpHidSdio_RegMap_t;

struct pHidSdio_RegMap_t {
    uint8_t HostToCardEvent;
    uint8_t HostIntCause;
    uint8_t HostIntMask;
    uint8_t HostIntStatus;
    uint16_t RdBitMap;
    uint16_t WrBitMap;
    uint16_t RdLen[16];
    uint8_t HostTransferStatus;
    uint8_t reserved1[7];
    uint8_t CardToHostEvent;
    uint8_t reserved2[3];
    uint8_t CardIntMask;
    uint8_t reserved3[3];
    uint8_t CardIntStatus;
    uint8_t reserved4[3];
    uint8_t CardIntMode;
    uint8_t reserved5[3];
    uint32_t SqReadBase;
    uint32_t SqWriteBase;
    uint8_t RdIdx;
    uint8_t WrIdx;
    uint8_t DnldQueueWrPtr;
    uint8_t UpldQueueWrPtr;
    uint8_t DnldQueue[8];
    uint8_t UpldQueue[8];
    uint8_t ChipRev;
    uint8_t reserved6;
    uint8_t IPRev0;
    uint8_t IPRev1;
    uint8_t reserved7[4];
    uint16_t Scratch2;
    uint16_t Scratch1;
    uint8_t Ocr0;
    uint8_t Ocr1;
    uint8_t Ocr2;
    uint8_t Config;
    uint32_t Config2;
    uint32_t Debug;
    uint32_t DmaAddr;
    uint8_t IoPort[3];
};

typedef struct anon_struct.conflicta49b anon_struct.conflicta49b, *Panon_struct.conflicta49b;

struct anon_struct.conflicta49b {
    uint8_t HostToCardEvent;
    uint8_t HostIntCause;
    uint8_t HostIntMask;
    uint8_t HostIntStatus;
    uint16_t RdBitMap;
    uint16_t WrBitMap;
    uint16_t RdLen[16];
    uint8_t HostTransferStatus;
    uint8_t reserved1[7];
    uint8_t CardToHostEvent;
    uint8_t reserved2[3];
    uint8_t CardIntMask;
    uint8_t reserved3[3];
    uint8_t CardIntStatus;
    uint8_t reserved4[3];
    uint8_t CardIntMode;
    uint8_t reserved5[3];
    uint32_t SqReadBase;
    uint32_t SqWriteBase;
    uint8_t RdIdx;
    uint8_t WrIdx;
    uint8_t DnldQueueWrPtr;
    uint8_t UpldQueueWrPtr;
    uint8_t DnldQueue[8];
    uint8_t UpldQueue[8];
    uint8_t ChipRev;
    uint8_t reserved6;
    uint8_t IPRev0;
    uint8_t IPRev1;
    uint8_t reserved7[4];
    uint16_t Scratch2;
    uint16_t Scratch1;
    uint8_t Ocr0;
    uint8_t Ocr1;
    uint8_t Ocr2;
    uint8_t Config;
    uint32_t Config2;
    uint32_t Debug;
    uint32_t DmaAddr;
    uint8_t IoPort[3];
};

typedef enum SDIO_CMD_TYPE {
    IOCTL_GET_CONFIG=0,
    IOCTL_HID_GET_BLOCK_SIZE=1
} SDIO_CMD_TYPE;

typedef struct Efuse_Ldo11VoutSelTrim_Info_Type Efuse_Ldo11VoutSelTrim_Info_Type, *PEfuse_Ldo11VoutSelTrim_Info_Type;

struct Efuse_Ldo11VoutSelTrim_Info_Type {
    uint32_t sel_value:4;
    uint32_t parity:1;
    uint32_t en:1;
    uint32_t rsvd:26;
};

typedef struct Efuse_TxPower_Info_Type Efuse_TxPower_Info_Type, *PEfuse_TxPower_Info_Type;

struct Efuse_TxPower_Info_Type {
    uint32_t txpower:5;
    uint32_t parity:1;
    uint32_t en:1;
    uint32_t rsvd:25;
};

typedef struct Efuse_Ana_RC32M_Trim_Type Efuse_Ana_RC32M_Trim_Type, *PEfuse_Ana_RC32M_Trim_Type;

struct Efuse_Ana_RC32M_Trim_Type {
    uint32_t trimRc32mCodeFrExt:8;
    uint32_t trimRc32mCodeFrExtParity:1;
    uint32_t trimRc32mExtCodeEn:1;
    uint32_t reserved:22;
};

typedef struct Efuse_Ana_RC32K_Trim_Type Efuse_Ana_RC32K_Trim_Type, *PEfuse_Ana_RC32K_Trim_Type;

struct Efuse_Ana_RC32K_Trim_Type {
    uint32_t trimRc32kCodeFrExt:10;
    uint32_t trimRc32kCodeFrExtParity:1;
    uint32_t trimRc32kExtCodeEn:1;
    uint32_t reserved:20;
};

typedef struct Efuse_Capcode_Info_Type Efuse_Capcode_Info_Type, *PEfuse_Capcode_Info_Type;

struct Efuse_Capcode_Info_Type {
    uint32_t capCode:6;
    uint32_t parity:1;
    uint32_t en:1;
    uint32_t rsvd:24;
};

typedef enum EF_Ctrl_Sign_Type {
    EF_CTRL_SIGN_NONE=0,
    EF_CTRL_SIGN_RSA=1,
    EF_CTRL_SIGN_ECC=2
} EF_Ctrl_Sign_Type;

typedef struct Efuse_Device_Info_Type Efuse_Device_Info_Type, *PEfuse_Device_Info_Type;

struct Efuse_Device_Info_Type {
    uint32_t rsvd:22;
    uint32_t customerID:2;
    uint32_t rsvd_info:3;
    uint32_t memoryInfo:2;
    uint32_t coreInfo:1;
    uint32_t mcuInfo:1;
    uint32_t pinInfo:1;
};

typedef struct Efuse_TSEN_Refcode_Corner_Type Efuse_TSEN_Refcode_Corner_Type, *PEfuse_TSEN_Refcode_Corner_Type;

struct Efuse_TSEN_Refcode_Corner_Type {
    uint32_t tsenRefcodeCorner:12;
    uint32_t tsenRefcodeCornerParity:1;
    uint32_t tsenRefcodeCornerEn:1;
    uint32_t tsenRefcodeCornerVersion:1;
    uint32_t reserved:17;
};

typedef enum EF_Ctrl_SF_AES_Type {
    EF_CTRL_SF_AES_NONE=0,
    EF_CTRL_SF_AES_128=1,
    EF_CTRL_SF_AES_192=2,
    EF_CTRL_SF_AES_256=3
} EF_Ctrl_SF_AES_Type;

typedef struct Efuse_ADC_Gain_Coeff_Type Efuse_ADC_Gain_Coeff_Type, *PEfuse_ADC_Gain_Coeff_Type;

struct Efuse_ADC_Gain_Coeff_Type {
    uint32_t adcGainCoeff:12;
    uint32_t adcGainCoeffParity:1;
    uint32_t adcGainCoeffEn:1;
    uint32_t reserved:18;
};

typedef struct EF_Ctrl_Sec_Param_Type EF_Ctrl_Sec_Param_Type, *PEF_Ctrl_Sec_Param_Type;

typedef enum EF_Ctrl_Dbg_Mode_Type {
    EF_CTRL_DBG_OPEN=0,
    EF_CTRL_DBG_PASSWORD=1,
    EF_CTRL_DBG_CLOSE=4
} EF_Ctrl_Dbg_Mode_Type;

struct EF_Ctrl_Sec_Param_Type {
    enum EF_Ctrl_Dbg_Mode_Type ef_dbg_mode;
    uint8_t ef_dbg_jtag_0_dis;
    uint8_t ef_sboot_en;
    uint8_t ef_no_hd_boot_en;
};

typedef struct SFlash_Sec_Reg_Cfg SFlash_Sec_Reg_Cfg, *PSFlash_Sec_Reg_Cfg;

struct SFlash_Sec_Reg_Cfg {
    uint8_t eraseCmd;
    uint8_t programCmd;
    uint8_t readCmd;
    uint8_t enterSecOptCmd;
    uint8_t exitSecOptCmd;
    uint8_t blockNum;
    undefined field6_0x6;
    undefined field7_0x7;
    uint8_t * data;
    uint32_t addr;
    uint32_t len;
};

typedef enum xz_mode {
    XZ_SINGLE=0,
    XZ_PREALLOC=1,
    XZ_DYNALLOC=2
} xz_mode;

typedef enum xz_ret {
    XZ_OK=0,
    XZ_STREAM_END=1,
    XZ_UNSUPPORTED_CHECK=2,
    XZ_MEM_ERROR=3,
    XZ_MEMLIMIT_ERROR=4,
    XZ_FORMAT_ERROR=5,
    XZ_OPTIONS_ERROR=6,
    XZ_DATA_ERROR=7,
    XZ_BUF_ERROR=8
} xz_ret;

typedef struct xz_buf xz_buf, *Pxz_buf;

struct xz_buf {
    uint8_t * in;
    size_t in_pos;
    size_t in_size;
    uint8_t * out;
    size_t out_pos;
    size_t out_size;
};

typedef enum xz_check {
    XZ_CHECK_NONE=0,
    XZ_CHECK_CRC32=1,
    XZ_CHECK_CRC64=4,
    XZ_CHECK_SHA256=10
} xz_check;

typedef ulonglong uint64_t;

typedef uint64_t vli_type;

typedef struct GLB_GPIO_Cfg_Type GLB_GPIO_Cfg_Type, *PGLB_GPIO_Cfg_Type;

struct GLB_GPIO_Cfg_Type {
    uint8_t gpioPin;
    uint8_t gpioFun;
    uint8_t gpioMode;
    uint8_t pullType;
    uint8_t drive;
    uint8_t smtCtrl;
};

typedef enum GLB_GPIO_FUNC_Type {
    GPIO_FUN_SDIO=1,
    GPIO_FUN_FLASH=2,
    GPIO_FUN_SPI=4,
    GPIO_FUN_I2C=6,
    GPIO_FUN_UART=7,
    GPIO_FUN_PWM=8,
    GPIO_FUN_EXT_PA=9,
    GPIO_FUN_ANALOG=10,
    GPIO_FUN_SWGPIO=11,
    GPIO_FUN_JTAG=14
} GLB_GPIO_FUNC_Type;

typedef enum GLB_GPIO_Type {
    GLB_GPIO_PIN_0=0,
    GLB_GPIO_PIN_1=1,
    GLB_GPIO_PIN_2=2,
    GLB_GPIO_PIN_3=3,
    GLB_GPIO_PIN_4=4,
    GLB_GPIO_PIN_5=5,
    GLB_GPIO_PIN_6=6,
    GLB_GPIO_PIN_7=7,
    GLB_GPIO_PIN_8=8,
    GLB_GPIO_PIN_9=9,
    GLB_GPIO_PIN_10=10,
    GLB_GPIO_PIN_11=11,
    GLB_GPIO_PIN_12=12,
    GLB_GPIO_PIN_13=13,
    GLB_GPIO_PIN_14=14,
    GLB_GPIO_PIN_15=15,
    GLB_GPIO_PIN_16=16,
    GLB_GPIO_PIN_17=17,
    GLB_GPIO_PIN_18=18,
    GLB_GPIO_PIN_19=19,
    GLB_GPIO_PIN_20=20,
    GLB_GPIO_PIN_21=21,
    GLB_GPIO_PIN_22=22,
    GLB_GPIO_PIN_MAX=23
} GLB_GPIO_Type;

typedef struct sys_clk_cfg_t sys_clk_cfg_t, *Psys_clk_cfg_t;

struct sys_clk_cfg_t {
    uint8_t xtal_type;
    uint8_t pll_clk;
    uint8_t hclk_div;
    uint8_t bclk_div;
    uint8_t flash_clk_type;
    uint8_t flash_clk_div;
    uint8_t rsvd[2];
};

typedef struct boot_clk_cfg_t boot_clk_cfg_t, *Pboot_clk_cfg_t;

struct boot_clk_cfg_t {
    uint32_t magiccode;
    struct sys_clk_cfg_t cfg;
    uint32_t crc32;
};

typedef union anon_union.conflict6ea7 anon_union.conflict6ea7, *Panon_union.conflict6ea7;

union anon_union.conflict6ea7 {
    uint32_t segment_cnt;
    uint32_t img_len;
};

typedef struct boot_cpu_cfg_t boot_cpu_cfg_t, *Pboot_cpu_cfg_t;

struct boot_cpu_cfg_t {
    uint32_t bootenable;
    uint32_t boot_magic;
    uint32_t flash_boot_offset;
    uint32_t msp_store_addr;
    uint32_t pc_store_addr;
    uint32_t dbg_info_addr;
    uint32_t dbg_log_addr;
    uint32_t dbg_exception_log_addr;
    uint32_t pkhash_src;
};

typedef struct boot_flash_cfg_t boot_flash_cfg_t, *Pboot_flash_cfg_t;

struct boot_flash_cfg_t {
    uint32_t magiccode;
    struct SPI_Flash_Cfg_Type cfg;
    uint32_t crc32;
};

typedef union anon_union.conflict6e83 anon_union.conflict6e83, *Panon_union.conflict6e83;

typedef struct anon_struct.conflict6da8 anon_struct.conflict6da8, *Panon_struct.conflict6da8;

struct anon_struct.conflict6da8 {
    uint32_t sign:2;
    uint32_t encrypt_type:2;
    uint32_t key_sel:2;
    uint32_t rsvd6_7:2;
    uint32_t no_segment:1;
    uint32_t cache_select:1;
    uint32_t notload_in_bootrom:1;
    uint32_t aes_region_lock:1;
    uint32_t cache_way_disable:4;
    uint32_t crc_ignore:1;
    uint32_t hash_ignore:1;
    uint32_t halt_ap:1;
    uint32_t rsvd19_31:13;
};

union anon_union.conflict6e83 {
    struct anon_struct.conflict6da8 bval;
    uint32_t wval;
};

typedef struct image_cfg_t image_cfg_t, *Pimage_cfg_t;

typedef union anon_union.conflict6ea7_for_img_segment_info anon_union.conflict6ea7_for_img_segment_info, *Panon_union.conflict6ea7_for_img_segment_info;

typedef union anon_union.conflict6eca_for_img_start anon_union.conflict6eca_for_img_start, *Panon_union.conflict6eca_for_img_start;

union anon_union.conflict6ea7_for_img_segment_info {
    uint32_t segment_cnt;
    uint32_t img_len;
};

union anon_union.conflict6eca_for_img_start {
    uint32_t ramaddr;
    uint32_t flashoffset;
};

struct image_cfg_t {
    uint8_t encrypt_type;
    uint8_t sign_type;
    uint8_t key_sel;
    uint8_t img_valid;
    uint8_t no_segment;
    uint8_t cache_select;
    uint8_t cache_way_disable;
    uint8_t hash_ignore;
    uint8_t aes_region_lock;
    uint8_t halt_ap;
    uint8_t r[2];
    union anon_union.conflict6ea7_for_img_segment_info img_segment_info;
    uint32_t mspval;
    uint32_t entrypoint;
    union anon_union.conflict6eca_for_img_start img_start;
    uint32_t sig_len;
    uint32_t sig_len2;
    uint32_t deallen;
    uint32_t maxinputlen;
};

typedef union anon_union.conflict6eca anon_union.conflict6eca, *Panon_union.conflict6eca;

union anon_union.conflict6eca {
    uint32_t ramaddr;
    uint32_t flashoffset;
};

typedef struct bootheader_t bootheader_t, *Pbootheader_t;

typedef union anon_union.conflict6e83_for_bootcfg anon_union.conflict6e83_for_bootcfg, *Panon_union.conflict6e83_for_bootcfg;

union anon_union.conflict6e83_for_bootcfg {
    struct anon_struct.conflict6da8 bval;
    uint32_t wval;
};

struct bootheader_t {
    uint32_t magiccode;
    uint32_t rivison;
    struct boot_flash_cfg_t flashCfg;
    struct boot_clk_cfg_t clkCfg;
    union anon_union.conflict6e83_for_bootcfg bootcfg;
    union anon_union.conflict6ea7_for_img_segment_info img_segment_info;
    uint32_t bootentry;
    union anon_union.conflict6eca_for_img_start img_start;
    uint8_t hash[32];
    uint32_t rsv1;
    uint32_t rsv2;
    uint32_t crc32;
};

typedef struct PDS_CTL2_Type PDS_CTL2_Type, *PPDS_CTL2_Type;

struct PDS_CTL2_Type {
    uint32_t forceCpuPwrOff:1;
    uint32_t rsv1:1;
    uint32_t forceWbPwrOff:1;
    uint32_t rsv3:1;
    uint32_t forceCpuIsoPwrOff:1;
    uint32_t rsv5:1;
    uint32_t forceWbIsoPwrOff:1;
    uint32_t rsv7:1;
    uint32_t forceCpuPdsRst:1;
    uint32_t rsv9:1;
    uint32_t forceWbPdsRst:1;
    uint32_t rsv11:1;
    uint32_t forceCpuMemStby:1;
    uint32_t rsv13:1;
    uint32_t forceWbMemStby:1;
    uint32_t rsv15:1;
    uint32_t forceCpuGateClk:1;
    uint32_t rsv17:1;
    uint32_t forceWbGateClk:1;
    uint32_t rsv19_31:12;
};

typedef enum PDS_PLL_CLK_Type {
    PDS_PLL_CLK_480M=0,
    PDS_PLL_CLK_240M=1,
    PDS_PLL_CLK_192M=2,
    PDS_PLL_CLK_160M=3,
    PDS_PLL_CLK_120M=4,
    PDS_PLL_CLK_96M=5,
    PDS_PLL_CLK_80M=6,
    PDS_PLL_CLK_48M=7,
    PDS_PLL_CLK_32M=8
} PDS_PLL_CLK_Type;

typedef struct PDS_DEFAULT_LV_CFG_Type PDS_DEFAULT_LV_CFG_Type, *PPDS_DEFAULT_LV_CFG_Type;

typedef struct PDS_CTL_Type PDS_CTL_Type, *PPDS_CTL_Type;

typedef struct PDS_CTL3_Type PDS_CTL3_Type, *PPDS_CTL3_Type;

typedef struct PDS_CTL4_Type PDS_CTL4_Type, *PPDS_CTL4_Type;

struct PDS_CTL4_Type {
    uint32_t cpuPwrOff:1;
    uint32_t cpuRst:1;
    uint32_t cpuMemStby:1;
    uint32_t cpuGateClk:1;
    uint32_t rsv4_11:8;
    uint32_t WbPwrOff:1;
    uint32_t WbRst:1;
    uint32_t WbMemStby:1;
    uint32_t WbGateClk:1;
    uint32_t rsv16_23:8;
    uint32_t MiscPwrOff:1;
    uint32_t MiscRst:1;
    uint32_t MiscMemStby:1;
    uint32_t MiscGateClk:1;
    uint32_t rsv28_31:4;
};

struct PDS_CTL3_Type {
    uint32_t rsv0:1;
    uint32_t forceMiscPwrOff:1;
    uint32_t rsv2_3:2;
    uint32_t forceMiscIsoEn:1;
    uint32_t rsv5_6:2;
    uint32_t forceMiscPdsRst:1;
    uint32_t rsv8_9:2;
    uint32_t forceMiscMemStby:1;
    uint32_t rsv11_12:2;
    uint32_t forceMiscGateClk:1;
    uint32_t rsv14_23:10;
    uint32_t CpuIsoEn:1;
    uint32_t rsv25_26:2;
    uint32_t WbIsoEn:1;
    uint32_t rsv28_29:2;
    uint32_t MiscIsoEn:1;
    uint32_t rsv31:1;
};

struct PDS_CTL_Type {
    uint32_t pdsStart:1;
    uint32_t sleepForever:1;
    uint32_t xtalForceOff:1;
    uint32_t saveWiFiState:1;
    uint32_t dcdc18Off:1;
    uint32_t bgSysOff:1;
    uint32_t rsv6_7:2;
    uint32_t clkOff:1;
    uint32_t memStby:1;
    uint32_t rsv10:1;
    uint32_t isolation:1;
    uint32_t waitXtalRdy:1;
    uint32_t pdsPwrOff:1;
    uint32_t xtalOff:1;
    uint32_t socEnbForceOn:1;
    uint32_t pdsRstSocEn:1;
    uint32_t pdsRC32mOn:1;
    uint32_t pdsLdoVselEn:1;
    uint32_t rsv19_20:2;
    uint32_t wfiMask:1;
    uint32_t ldo11Off:1;
    uint32_t rsv23:1;
    uint32_t pdsLdoVol:4;
    uint32_t pdsCtlRfSel:2;
    uint32_t pdsCtlPllSel:2;
};

struct PDS_DEFAULT_LV_CFG_Type {
    struct PDS_CTL_Type pdsCtl;
    struct PDS_CTL2_Type pdsCtl2;
    struct PDS_CTL3_Type pdsCtl3;
    struct PDS_CTL4_Type pdsCtl4;
};

typedef struct PDS_RAM_CFG_Type PDS_RAM_CFG_Type, *PPDS_RAM_CFG_Type;

struct PDS_RAM_CFG_Type {
    uint32_t PDS_RAM_CFG_0KB_16KB_CPU_RAM_RET:1;
    uint32_t PDS_RAM_CFG_16KB_32KB_CPU_RAM_RET:1;
    uint32_t PDS_RAM_CFG_32KB_48KB_CPU_RAM_RET:1;
    uint32_t PDS_RAM_CFG_48KB_64KB_CPU_RAM_RET:1;
    uint32_t PDS_RAM_CFG_0KB_16KB_CPU_RAM_SLP:1;
    uint32_t PDS_RAM_CFG_16KB_32KB_CPU_RAM_SLP:1;
    uint32_t PDS_RAM_CFG_32KB_48KB_CPU_RAM_SLP:1;
    uint32_t PDS_RAM_CFG_48KB_64KB_CPU_RAM_SLP:1;
    uint32_t PDS_RAM_CFG_RSV:24;
};

typedef enum PDS_PLL_XTAL_Type {
    PDS_PLL_XTAL_NONE=0,
    PDS_PLL_XTAL_24M=1,
    PDS_PLL_XTAL_32M=2,
    PDS_PLL_XTAL_38P4M=3,
    PDS_PLL_XTAL_40M=4,
    PDS_PLL_XTAL_26M=5,
    PDS_PLL_XTAL_RC32M=6
} PDS_PLL_XTAL_Type;

typedef enum HBN_PIR_HPF_Type {
    HBN_PIR_HPF_METHOD0=0,
    HBN_PIR_HPF_METHOD1=1,
    HBN_PIR_HPF_METHOD2=2
} HBN_PIR_HPF_Type;

typedef enum HBN_OUT0_INT_Type {
    HBN_OUT0_INT_GPIO7=0,
    HBN_OUT0_INT_GPIO8=1,
    HBN_OUT0_INT_RTC=2
} HBN_OUT0_INT_Type;

typedef enum HBN_XCLK_CLK_Type {
    HBN_XCLK_CLK_RC32M=0,
    HBN_XCLK_CLK_XTAL=1
} HBN_XCLK_CLK_Type;

typedef enum HBN_BOR_THRES_Type {
    HBN_BOR_THRES_2P0V=0,
    HBN_BOR_THRES_2P4V=1
} HBN_BOR_THRES_Type;

typedef struct HBN_APP_CFG_Type HBN_APP_CFG_Type, *PHBN_APP_CFG_Type;

typedef enum HBN_GPIO_INT_Trigger_Type {
    HBN_GPIO_INT_TRIGGER_SYNC_FALLING_EDGE=0,
    HBN_GPIO_INT_TRIGGER_SYNC_RISING_EDGE=1,
    HBN_GPIO_INT_TRIGGER_SYNC_LOW_LEVEL=2,
    HBN_GPIO_INT_TRIGGER_SYNC_HIGH_LEVEL=3,
    HBN_GPIO_INT_TRIGGER_ASYNC_FALLING_EDGE=4,
    HBN_GPIO_INT_TRIGGER_ASYNC_RISING_EDGE=5,
    HBN_GPIO_INT_TRIGGER_ASYNC_LOW_LEVEL=6,
    HBN_GPIO_INT_TRIGGER_ASYNC_HIGH_LEVEL=7
} HBN_GPIO_INT_Trigger_Type;

typedef enum HBN_LEVEL_Type {
    HBN_LEVEL_0=0,
    HBN_LEVEL_1=1,
    HBN_LEVEL_2=2,
    HBN_LEVEL_3=3
} HBN_LEVEL_Type;

typedef enum HBN_LDO_LEVEL_Type {
    HBN_LDO_LEVEL_0P60V=0,
    HBN_LDO_LEVEL_0P65V=1,
    HBN_LDO_LEVEL_0P70V=2,
    HBN_LDO_LEVEL_0P75V=3,
    HBN_LDO_LEVEL_0P80V=4,
    HBN_LDO_LEVEL_0P85V=5,
    HBN_LDO_LEVEL_0P90V=6,
    HBN_LDO_LEVEL_0P95V=7,
    HBN_LDO_LEVEL_1P00V=8,
    HBN_LDO_LEVEL_1P05V=9,
    HBN_LDO_LEVEL_1P10V=10,
    HBN_LDO_LEVEL_1P15V=11,
    HBN_LDO_LEVEL_1P20V=12,
    HBN_LDO_LEVEL_1P25V=13,
    HBN_LDO_LEVEL_1P30V=14,
    HBN_LDO_LEVEL_1P35V=15
} HBN_LDO_LEVEL_Type;

struct HBN_APP_CFG_Type {
    uint8_t useXtal32k;
    undefined field1_0x1;
    undefined field2_0x2;
    undefined field3_0x3;
    uint32_t sleepTime;
    uint8_t gpioWakeupSrc;
    enum HBN_GPIO_INT_Trigger_Type gpioTrigType;
    undefined field7_0xa;
    undefined field8_0xb;
    struct SPI_Flash_Cfg_Type * flashCfg;
    enum HBN_LEVEL_Type hbnLevel;
    enum HBN_LDO_LEVEL_Type ldoLevel;
    undefined field12_0x12;
    undefined field13_0x13;
};

typedef struct HBN_BOR_CFG_Type HBN_BOR_CFG_Type, *PHBN_BOR_CFG_Type;

struct HBN_BOR_CFG_Type {
    uint8_t enableBor;
    uint8_t enableBorInt;
    uint8_t borThreshold;
    uint8_t enablePorInBor;
};

typedef enum HBN_PIR_LPF_Type {
    HBN_PIR_LPF_DIV1=0,
    HBN_PIR_LPF_DIV2=1
} HBN_PIR_LPF_Type;

typedef struct HBN_PIR_INT_CFG_Type HBN_PIR_INT_CFG_Type, *PHBN_PIR_INT_CFG_Type;

struct HBN_PIR_INT_CFG_Type {
    enum BL_Fun_Type lowIntEn;
    enum BL_Fun_Type highIntEn;
};

typedef enum HBN_32K_CLK_Type {
    HBN_32K_RC=0,
    HBN_32K_XTAL=1,
    HBN_32K_DIG=3
} HBN_32K_CLK_Type;

typedef enum HBN_ROOT_CLK_Type {
    HBN_ROOT_CLK_RC32M=0,
    HBN_ROOT_CLK_XTAL=1,
    HBN_ROOT_CLK_PLL=2
} HBN_ROOT_CLK_Type;

typedef enum HBN_UART_CLK_Type {
    HBN_UART_CLK_FCLK=0,
    HBN_UART_CLK_160M=1
} HBN_UART_CLK_Type;

typedef enum HBN_INT_Type {
    HBN_INT_GPIO7=0,
    HBN_INT_GPIO8=1,
    HBN_INT_RTC=16,
    HBN_INT_PIR=17,
    HBN_INT_BOR=18,
    HBN_INT_ACOMP0=20,
    HBN_INT_ACOMP1=22
} HBN_INT_Type;

typedef enum HBN_OUT1_INT_Type {
    HBN_OUT1_INT_PIR=0,
    HBN_OUT1_INT_BOR=1,
    HBN_OUT1_INT_ACOMP0=2,
    HBN_OUT1_INT_ACOMP1=3
} HBN_OUT1_INT_Type;

typedef enum HBN_ACOMP_INT_EDGE_Type {
    HBN_ACOMP_INT_EDGE_POSEDGE=0,
    HBN_ACOMP_INT_EDGE_NEGEDGE=1
} HBN_ACOMP_INT_EDGE_Type;

typedef enum HBN_BOR_MODE_Type {
    HBN_BOR_MODE_POR_INDEPENDENT=0,
    HBN_BOR_MODE_POR_RELEVANT=1
} HBN_BOR_MODE_Type;

typedef enum HBN_RTC_INT_Delay_Type {
    HBN_RTC_INT_DELAY_32T=0,
    HBN_RTC_INT_DELAY_0T=1
} HBN_RTC_INT_Delay_Type;

typedef enum BL_Mask_Type {
    UNMASK=0,
    MASK=1
} BL_Mask_Type;

typedef enum BL_Sts_Type {
    RESET=0,
    SET=1
} BL_Sts_Type;

typedef enum BL_Err_Type {
    SUCCESS=0,
    ERROR=1,
    TIMEOUT=2
} BL_Err_Type;

typedef enum anon_enum_8.conflicta8 {
    BL_AHB_SLAVE1_GLB=0,
    DISABLE=0,
    GLB_EM_0KB=0,
    GLB_GPIO_PIN_0=0,
    GLB_MTIMER_CLK_BCLK=0,
    GLB_PLL_XTAL_NONE=0,
    GLB_SFLASH_CLK_120M=0,
    GLB_SYS_CLK_RC32M=0,
    GLB_UART_SIG_0=0,
    HBN_32K_RC=0,
    HBN_ROOT_CLK_RC32M=0,
    HBN_UART_CLK_FCLK=0,
    RESET=0,
    SEC_ENG_SHA256=0,
    SF_CTRL_ADDR_1_LINE=0,
    SF_CTRL_AES_128BITS=0,
    SF_CTRL_DATA_1_LINE=0,
    SF_CTRL_NIO_MODE=0,
    SF_CTRL_READ=0,
    SUCCESS=0,
    UART_AUTOBAUD_0X55=0,
    UART_INT_TX_END=0,
    UART_TX=0,
    UNMASK=0,
    BL_AHB_SLAVE1_RF=1,
    ENABLE=1,
    ERROR=1,
    GLB_GPIO_PIN_1=1,
    GLB_MTIMER_CLK_32K=1,
    GLB_PLL_XTAL_24M=1,
    GLB_SFLASH_CLK_XTAL=1,
    GLB_SYS_CLK_XTAL=1,
    GLB_UART_SIG_1=1,
    HBN_32K_XTAL=1,
    HBN_ROOT_CLK_XTAL=1,
    HBN_UART_CLK_160M=1,
    MASK=1,
    SEC_ENG_SHA224=1,
    SET=1,
    SF_CTRL_ADDR_2_LINES=1,
    SF_CTRL_AES_256BITS=1,
    SF_CTRL_DATA_2_LINES=1,
    SF_CTRL_DO_MODE=1,
    SF_CTRL_WRITE=1,
    UART_AUTOBAUD_STARTBIT=1,
    UART_INT_RX_END=1,
    UART_RX=1,
    BL_AHB_SLAVE1_GPIP_PHY_AGC=2,
    GLB_GPIO_PIN_2=2,
    GLB_PLL_XTAL_32M=2,
    GLB_SFLASH_CLK_48M=2,
    GLB_SYS_CLK_PLL48M=2,
    GLB_UART_SIG_2=2,
    HBN_ROOT_CLK_PLL=2,
    SEC_ENG_SHA1=2,
    SF_CTRL_ADDR_4_LINES=2,
    SF_CTRL_AES_192BITS=2,
    SF_CTRL_DATA_4_LINES=2,
    SF_CTRL_QO_MODE=2,
    TIMEOUT=2,
    UART_INT_TX_FIFO_REQ=2,
    UART_TXRX=2,
    BL_AHB_SLAVE1_SEC_DBG=3,
    GLB_EM_8KB=3,
    GLB_GPIO_PIN_3=3,
    GLB_PLL_XTAL_38P4M=3,
    GLB_SFLASH_CLK_80M=3,
    GLB_SYS_CLK_PLL120M=3,
    GLB_UART_SIG_3=3,
    HBN_32K_DIG=3,
    MSOFT_IRQn=3,
    SEC_ENG_SHA1_RSVD=3,
    SF_CTRL_AES_128BITS_DOUBLE_KEY=3,
    SF_CTRL_DIO_MODE=3,
    UART_INT_RX_FIFO_REQ=3,
    BL_AHB_SLAVE1_SEC=4,
    GLB_GPIO_PIN_4=4,
    GLB_PLL_XTAL_40M=4,
    GLB_SFLASH_CLK_BCLK=4,
    GLB_SYS_CLK_PLL160M=4,
    GLB_UART_SIG_4=4,
    SF_CTRL_QIO_MODE=4,
    UART_INT_RTO=4,
    BL_AHB_SLAVE1_TZ1=5,
    GLB_GPIO_PIN_5=5,
    GLB_PLL_XTAL_26M=5,
    GLB_SFLASH_CLK_96M=5,
    GLB_SYS_CLK_PLL192M=5,
    GLB_UART_SIG_5=5,
    UART_INT_PCE=5,
    BL_AHB_SLAVE1_TZ2=6,
    GLB_GPIO_PIN_6=6,
    GLB_PLL_XTAL_RC32M=6,
    GLB_UART_SIG_6=6,
    UART_INT_TX_FER=6,
    BL_AHB_SLAVE1_EFUSE=7,
    GLB_GPIO_PIN_7=7,
    GLB_UART_SIG_7=7,
    MTIME_IRQn=7,
    UART_INT_RX_FER=7,
    BL_AHB_SLAVE1_CCI=8,
    GLB_GPIO_PIN_8=8,
    UART_INT_ALL=8,
    BL_AHB_SLAVE1_L1C=9,
    GLB_GPIO_PIN_9=9,
    BL_AHB_SLAVE1_RSVD0A=10,
    GLB_GPIO_PIN_10=10,
    BL_AHB_SLAVE1_SFC=11,
    GLB_GPIO_PIN_11=11,
    MEXT_IRQn=11,
    BL_AHB_SLAVE1_DMA=12,
    CLIC_SOFT_PEND_IRQn=12,
    GLB_GPIO_PIN_12=12,
    BL_AHB_SLAVE1_SDU=13,
    GLB_GPIO_PIN_13=13,
    BL_AHB_SLAVE1_PDS_HBN_AON_HBNRAM=14,
    GLB_GPIO_PIN_14=14,
    BL_AHB_SLAVE1_RSVD0F=15,
    GLB_EM_16KB=15,
    GLB_GPIO_PIN_15=15,
    BL_AHB_SLAVE1_UART0=16,
    BMX_ERR_IRQn=16,
    GLB_GPIO_PIN_16=16,
    BL_AHB_SLAVE1_UART1=17,
    BMX_TO_IRQn=17,
    GLB_GPIO_PIN_17=17,
    BL_AHB_SLAVE1_SPI=18,
    GLB_GPIO_PIN_18=18,
    L1C_BMX_ERR_IRQn=18,
    BL_AHB_SLAVE1_I2C=19,
    GLB_GPIO_PIN_19=19,
    L1C_BMX_TO_IRQn=19,
    BL_AHB_SLAVE1_PWM=20,
    GLB_GPIO_PIN_20=20,
    SEC_BMX_ERR_IRQn=20,
    BL_AHB_SLAVE1_TMR=21,
    GLB_GPIO_PIN_21=21,
    RF_TOP_INT0_IRQn=21,
    BL_AHB_SLAVE1_IRR=22,
    GLB_GPIO_PIN_22=22,
    RF_TOP_INT1_IRQn=22,
    BL_AHB_SLAVE1_CKS=23,
    GLB_GPIO_PIN_MAX=23,
    SDIO_IRQn=23,
    BL_AHB_SLAVE1_MAX=24,
    DMA_BMX_ERR_IRQn=24,
    SEC_GMAC_IRQn=25,
    SEC_CDET_IRQn=26,
    SEC_PKA_IRQn=27,
    SEC_TRNG_IRQn=28,
    SEC_AES_IRQn=29,
    SEC_SHA_IRQn=30,
    DMA_ALL_IRQn=31,
    RESERVED0=32,
    RESERVED1=33,
    RESERVED2=34,
    IRTX_IRQn=35,
    IRRX_IRQn=36,
    RESERVED3=37,
    RESERVED4=38,
    SF_CTRL_IRQn=39,
    RESERVED5=40,
    GPADC_DMA_IRQn=41,
    EFUSE_IRQn=42,
    SPI_IRQn=43,
    RESERVED6=44,
    UART0_IRQn=45,
    UART1_IRQn=46,
    RESERVED7=47,
    I2C_IRQn=48,
    RESERVED8=49,
    PWM_IRQn=50,
    RESERVED9=51,
    TIMER_CH0_IRQn=52,
    TIMER_CH1_IRQn=53,
    TIMER_WDT_IRQn=54,
    RESERVED10=55,
    RESERVED11=56,
    RESERVED12=57,
    RESERVED13=58,
    RESERVED14=59,
    GPIO_INT0_IRQn=60,
    RESERVED16=61,
    RESERVED17=62,
    RESERVED18=63,
    RESERVED19=64,
    RESERVED20=65,
    PDS_WAKEUP_IRQn=66,
    HBN_OUT0_IRQn=67,
    HBN_OUT1_IRQn=68,
    BOR_IRQn=69,
    WIFI_IRQn=70,
    BZ_PHY_IRQn=71,
    BLE_IRQn=72,
    MAC_TXRX_TIMER_IRQn=73,
    MAC_TXRX_MISC_IRQn=74,
    MAC_RX_TRG_IRQn=75,
    MAC_TX_TRG_IRQn=76,
    MAC_GEN_IRQn=77,
    MAC_PORT_TRG_IRQn=78,
    WIFI_IPC_PUBLIC_IRQn=79,
    IRQn_LAST=80
} anon_enum_8.conflicta8;

typedef struct anon_struct.conflictd3d3 anon_struct.conflictd3d3, *Panon_struct.conflictd3d3;

typedef struct xz_dec_hash xz_dec_hash, *Pxz_dec_hash;

struct xz_dec_hash {
    vli_type unpadded;
    vli_type uncompressed;
    uint32_t crc32;
    undefined field3_0x14;
    undefined field4_0x15;
    undefined field5_0x16;
    undefined field6_0x17;
};

struct anon_struct.conflictd3d3 {
    vli_type compressed;
    vli_type uncompressed;
    vli_type count;
    struct xz_dec_hash hash;
};

typedef struct xz_dec xz_dec, *Pxz_dec;

typedef enum xz_dec_tmp_sequence1 {
    SEQ_STREAM_HEADER=0,
    SEQ_BLOCK_START=1,
    SEQ_BLOCK_HEADER=2,
    SEQ_BLOCK_UNCOMPRESS=3,
    SEQ_BLOCK_PADDING=4,
    SEQ_BLOCK_CHECK=5,
    SEQ_INDEX=6,
    SEQ_INDEX_PADDING=7,
    SEQ_INDEX_CRC32=8,
    SEQ_STREAM_FOOTER=9
} xz_dec_tmp_sequence1;

typedef struct anon_struct.conflictd3a2_for_block_header anon_struct.conflictd3a2_for_block_header, *Panon_struct.conflictd3a2_for_block_header;

typedef struct anon_struct.conflictd3d3_for_block anon_struct.conflictd3d3_for_block, *Panon_struct.conflictd3d3_for_block;

typedef struct anon_struct.conflictd411_for_index anon_struct.conflictd411_for_index, *Panon_struct.conflictd411_for_index;

typedef struct anon_struct.conflictd44f_for_temp anon_struct.conflictd44f_for_temp, *Panon_struct.conflictd44f_for_temp;

typedef struct xz_dec_lzma2 xz_dec_lzma2, *Pxz_dec_lzma2;

typedef enum xz_dec_tmp_sequence2 {
    SEQ_INDEX_COUNT=0,
    SEQ_INDEX_UNPADDED=1,
    SEQ_INDEX_UNCOMPRESSED=2
} xz_dec_tmp_sequence2;

typedef struct rc_dec rc_dec, *Prc_dec;

typedef struct dictionary dictionary, *Pdictionary;

typedef struct lzma2_dec lzma2_dec, *Plzma2_dec;

typedef struct lzma_dec lzma_dec, *Plzma_dec;

typedef struct anon_struct.conflictbb9d_for_temp anon_struct.conflictbb9d_for_temp, *Panon_struct.conflictbb9d_for_temp;

typedef enum lzma2_seq {
    SEQ_CONTROL=0,
    SEQ_UNCOMPRESSED_1=1,
    SEQ_UNCOMPRESSED_2=2,
    SEQ_COMPRESSED_0=3,
    SEQ_COMPRESSED_1=4,
    SEQ_PROPERTIES=5,
    SEQ_LZMA_PREPARE=6,
    SEQ_LZMA_RUN=7,
    SEQ_COPY=8
} lzma2_seq;

typedef enum lzma_state {
    STATE_LIT_LIT=0,
    STATE_MATCH_LIT_LIT=1,
    STATE_REP_LIT_LIT=2,
    STATE_SHORTREP_LIT_LIT=3,
    STATE_MATCH_LIT=4,
    STATE_REP_LIT=5,
    STATE_SHORTREP_LIT=6,
    STATE_LIT_MATCH=7,
    STATE_LIT_LONGREP=8,
    STATE_LIT_SHORTREP=9,
    STATE_NONLIT_MATCH=10,
    STATE_NONLIT_REP=11
} lzma_state;

typedef struct lzma_len_dec lzma_len_dec, *Plzma_len_dec;

struct lzma_len_dec {
    uint16_t choice;
    uint16_t choice2;
    uint16_t low[16][8];
    uint16_t mid[16][8];
    uint16_t high[256];
};

struct lzma_dec {
    uint32_t rep0;
    uint32_t rep1;
    uint32_t rep2;
    uint32_t rep3;
    enum lzma_state state;
    undefined field5_0x11;
    undefined field6_0x12;
    undefined field7_0x13;
    uint32_t len;
    uint32_t lc;
    uint32_t literal_pos_mask;
    uint32_t pos_mask;
    uint16_t is_match[12][16];
    uint16_t is_rep[12];
    uint16_t is_rep0[12];
    uint16_t is_rep1[12];
    uint16_t is_rep2[12];
    uint16_t is_rep0_long[12][16];
    uint16_t dist_slot[4][64];
    uint16_t dist_special[114];
    uint16_t dist_align[16];
    struct lzma_len_dec match_len_dec;
    struct lzma_len_dec rep_len_dec;
    uint16_t literal[16][768];
};

struct rc_dec {
    uint32_t range;
    uint32_t code;
    uint32_t init_bytes_left;
    uint8_t * in;
    size_t in_pos;
    size_t in_limit;
};

struct lzma2_dec {
    enum lzma2_seq sequence;
    enum lzma2_seq next_sequence;
    undefined field2_0x2;
    undefined field3_0x3;
    uint32_t uncompressed;
    uint32_t compressed;
    _Bool need_dict_reset;
    _Bool need_props;
    undefined field8_0xe;
    undefined field9_0xf;
};

struct dictionary {
    uint8_t * buf;
    size_t start;
    size_t pos;
    size_t full;
    size_t limit;
    size_t end;
    uint32_t size;
    uint32_t size_max;
    uint32_t allocated;
    enum xz_mode mode;
    undefined field10_0x25;
    undefined field11_0x26;
    undefined field12_0x27;
};

struct anon_struct.conflictbb9d_for_temp {
    uint32_t size;
    uint8_t buf[63];
    undefined field2_0x43;
};

struct xz_dec_lzma2 {
    struct rc_dec rc;
    struct dictionary dict;
    struct lzma2_dec lzma2;
    struct lzma_dec lzma;
    struct anon_struct.conflictbb9d_for_temp temp;
};

struct anon_struct.conflictd411_for_index {
    enum xz_dec_tmp_sequence2 sequence;
    undefined field1_0x1;
    undefined field2_0x2;
    undefined field3_0x3;
    undefined field4_0x4;
    undefined field5_0x5;
    undefined field6_0x6;
    undefined field7_0x7;
    vli_type size;
    vli_type count;
    struct xz_dec_hash hash;
};

struct anon_struct.conflictd44f_for_temp {
    size_t pos;
    size_t size;
    uint8_t buf[1024];
};

struct anon_struct.conflictd3d3_for_block {
    vli_type compressed;
    vli_type uncompressed;
    vli_type count;
    struct xz_dec_hash hash;
};

struct anon_struct.conflictd3a2_for_block_header {
    vli_type compressed;
    vli_type uncompressed;
    uint32_t size;
    undefined field3_0x14;
    undefined field4_0x15;
    undefined field5_0x16;
    undefined field6_0x17;
};

struct xz_dec {
    enum xz_dec_tmp_sequence1 sequence;
    undefined field1_0x1;
    undefined field2_0x2;
    undefined field3_0x3;
    uint32_t pos;
    vli_type vli;
    size_t in_start;
    size_t out_start;
    uint32_t crc;
    enum xz_check check_type;
    enum xz_mode mode;
    _Bool allow_buf_error;
    undefined field12_0x1f;
    struct anon_struct.conflictd3a2_for_block_header block_header;
    struct anon_struct.conflictd3d3_for_block block;
    struct anon_struct.conflictd411_for_index index;
    struct anon_struct.conflictd44f_for_temp temp;
    struct xz_dec_lzma2 * lzma2;
    undefined field18_0x4a4;
    undefined field19_0x4a5;
    undefined field20_0x4a6;
    undefined field21_0x4a7;
};

typedef struct anon_struct.conflictd411 anon_struct.conflictd411, *Panon_struct.conflictd411;

struct anon_struct.conflictd411 {
    enum xz_dec_tmp_sequence2 sequence;
    undefined field1_0x1;
    undefined field2_0x2;
    undefined field3_0x3;
    undefined field4_0x4;
    undefined field5_0x5;
    undefined field6_0x6;
    undefined field7_0x7;
    vli_type size;
    vli_type count;
    struct xz_dec_hash hash;
};

typedef struct anon_struct.conflictd3a2 anon_struct.conflictd3a2, *Panon_struct.conflictd3a2;

struct anon_struct.conflictd3a2 {
    vli_type compressed;
    vli_type uncompressed;
    uint32_t size;
    undefined field3_0x14;
    undefined field4_0x15;
    undefined field5_0x16;
    undefined field6_0x17;
};

typedef struct anon_struct.conflictd44f anon_struct.conflictd44f, *Panon_struct.conflictd44f;

struct anon_struct.conflictd44f {
    size_t pos;
    size_t size;
    uint8_t buf[1024];
};

typedef enum anon_enum_16.conflict1b7af {
    ROM_API_INDEX_SFlash_Read_Reg_With_Cmd=-128,
    ROM_API_INDEX_SFlash_Write_Reg_With_Cmd=-127,
    ROM_API_INDEX_SFlash_Restore_From_Powerdown=-126,
    ROM_API_INDEX_SF_Cfg_Init_Ext_Flash_Gpio=-125,
    ROM_API_INDEX_SF_Cfg_Init_Internal_Flash_Gpio=-124,
    ROM_API_INDEX_SF_Cfg_Deinit_Ext_Flash_Gpio=-123,
    ROM_API_INDEX_SF_Cfg_Restore_GPIO17_Fun=-122,
    ROM_API_INDEX_SF_Cfg_Get_Flash_Cfg_Need_Lock=-121,
    ROM_API_INDEX_SF_Cfg_Init_Flash_Gpio=-120,
    ROM_API_INDEX_SF_Cfg_Flash_Identify=-119,
    ROM_API_INDEX_SF_Ctrl_Enable=-118,
    ROM_API_INDEX_SF_Ctrl_Select_Pad=-117,
    ROM_API_INDEX_SF_Ctrl_Set_Owner=-116,
    ROM_API_INDEX_SF_Ctrl_Disable=-115,
    ROM_API_INDEX_SF_Ctrl_AES_Enable_BE=-114,
    ROM_API_INDEX_SF_Ctrl_AES_Enable_LE=-113,
    ROM_API_INDEX_SF_Ctrl_AES_Set_Region=-112,
    ROM_API_INDEX_SF_Ctrl_AES_Set_Key=-111,
    ROM_API_INDEX_SF_Ctrl_AES_Set_Key_BE=-110,
    ROM_API_INDEX_SF_Ctrl_AES_Set_IV=-109,
    ROM_API_INDEX_SF_Ctrl_AES_Set_IV_BE=-108,
    ROM_API_INDEX_SF_Ctrl_AES_Enable=-107,
    ROM_API_INDEX_SF_Ctrl_AES_Disable=-106,
    ROM_API_INDEX_SF_Ctrl_Set_Flash_Image_Offset=-105,
    ROM_API_INDEX_SF_Ctrl_Get_Flash_Image_Offset=-104,
    ROM_API_INDEX_SF_Ctrl_Select_Clock=-103,
    ROM_API_INDEX_SF_Ctrl_SendCmd=-102,
    ROM_API_INDEX_SF_Ctrl_Icache_Set=-101,
    ROM_API_INDEX_SF_Ctrl_Icache2_Set=-100,
    ROM_API_INDEX_SF_Ctrl_GetBusyState=-99,
    ROM_API_INDEX_SF_Ctrl_Is_AES_Enable=-98,
    ROM_API_INDEX_SF_Ctrl_Get_Clock_Delay=-97,
    ROM_API_INDEX_SF_Ctrl_Set_Clock_Delay=-96,
    ROM_API_INDEX_XIP_SFlash_State_Save=-95,
    ROM_API_INDEX_XIP_SFlash_State_Restore=-94,
    ROM_API_INDEX_XIP_SFlash_Erase_Need_Lock=-93,
    ROM_API_INDEX_XIP_SFlash_Write_Need_Lock=-92,
    ROM_API_INDEX_XIP_SFlash_Read_Need_Lock=-91,
    ROM_API_INDEX_XIP_SFlash_GetJedecId_Need_Lock=-90,
    ROM_API_INDEX_XIP_SFlash_GetDeviceId_Need_Lock=-89,
    ROM_API_INDEX_XIP_SFlash_GetUniqueId_Need_Lock=-88,
    ROM_API_INDEX_XIP_SFlash_Read_Via_Cache_Need_Lock=-87,
    ROM_API_INDEX_XIP_SFlash_Read_With_Lock=-86,
    ROM_API_INDEX_XIP_SFlash_Write_With_Lock=-85,
    ROM_API_INDEX_XIP_SFlash_Erase_With_Lock=-84,
    ROM_API_INDEX_XIP_SFlash_Opt_Enter=-83,
    ROM_API_INDEX_XIP_SFlash_Opt_Exit=-82,
    ROM_API_INDEX_BFLB_Soft_CRC32=-81,
    ROM_API_INDEX_FUNC_EMPTY_START=-80,
    ROM_API_INDEX_VERSION=0,
    ROM_API_INDEX_RSVD_0=1,
    ROM_API_INDEX_RSVD_1=2,
    ROM_API_INDEX_RSVD_LAST=3,
    ROM_API_INDEX_AON_Power_On_MBG=4,
    ROM_API_INDEX_AON_Power_Off_MBG=5,
    ROM_API_INDEX_AON_Power_On_XTAL=6,
    ROM_API_INDEX_AON_Set_Xtal_CapCode=7,
    ROM_API_INDEX_AON_Get_Xtal_CapCode=8,
    ROM_API_INDEX_AON_Power_Off_XTAL=9,
    ROM_API_INDEX_AON_Power_On_BG=10,
    ROM_API_INDEX_AON_Power_Off_BG=11,
    ROM_API_INDEX_AON_Power_On_LDO11_SOC=12,
    ROM_API_INDEX_AON_Power_Off_LDO11_SOC=13,
    ROM_API_INDEX_AON_Power_On_LDO15_RF=14,
    ROM_API_INDEX_AON_Power_Off_LDO15_RF=15,
    ROM_API_INDEX_AON_Power_On_SFReg=16,
    ROM_API_INDEX_AON_Power_Off_SFReg=17,
    ROM_API_INDEX_AON_LowPower_Enter_PDS0=18,
    ROM_API_INDEX_AON_LowPower_Exit_PDS0=19,
    ROM_API_INDEX_ASM_Delay_Us=20,
    ROM_API_INDEX_BL602_Delay_US=21,
    ROM_API_INDEX_BL602_Delay_MS=22,
    ROM_API_INDEX_BL602_MemCpy=23,
    ROM_API_INDEX_BL602_MemCpy4=24,
    ROM_API_INDEX_BL602_MemCpy_Fast=25,
    ROM_API_INDEX_BL602_MemSet=26,
    ROM_API_INDEX_BL602_MemSet4=27,
    ROM_API_INDEX_BL602_MemCmp=28,
    ROM_API_INDEX_EF_Ctrl_Sw_AHB_Clk_0=29,
    ROM_API_INDEX_EF_Ctrl_Program_Efuse_0=30,
    ROM_API_INDEX_EF_Ctrl_Load_Efuse_R0=31,
    ROM_API_INDEX_EF_Ctrl_Busy=32,
    ROM_API_INDEX_EF_Ctrl_AutoLoad_Done=33,
    ROM_API_INDEX_EF_Ctrl_Get_Trim_Parity=34,
    ROM_API_INDEX_EF_Ctrl_Read_RC32M_Trim=35,
    ROM_API_INDEX_EF_Ctrl_Read_RC32K_Trim=36,
    ROM_API_INDEX_EF_Ctrl_Clear=37,
    ROM_API_INDEX_GLB_Get_Root_CLK_Sel=38,
    ROM_API_INDEX_GLB_Set_System_CLK_Div=39,
    ROM_API_INDEX_GLB_Get_BCLK_Div=40,
    ROM_API_INDEX_GLB_Get_HCLK_Div=41,
    ROM_API_INDEX_Update_SystemCoreClockWith_XTAL=42,
    ROM_API_INDEX_GLB_Set_System_CLK=43,
    ROM_API_INDEX_System_Core_Clock_Update_From_RC32M=44,
    ROM_API_INDEX_GLB_Set_SF_CLK=45,
    ROM_API_INDEX_GLB_Set_PKA_CLK_Sel=46,
    ROM_API_INDEX_GLB_SW_System_Reset=47,
    ROM_API_INDEX_GLB_SW_CPU_Reset=48,
    ROM_API_INDEX_GLB_SW_POR_Reset=49,
    ROM_API_INDEX_GLB_Select_Internal_Flash=50,
    ROM_API_INDEX_GLB_Select_External_Flash=51,
    ROM_API_INDEX_GLB_Deswap_Flash_Pin=52,
    ROM_API_INDEX_GLB_Swap_Flash_Pin=53,
    ROM_API_INDEX_GLB_GPIO_Init=54,
    ROM_API_INDEX_GLB_GPIO_OUTPUT_Enable=55,
    ROM_API_INDEX_GLB_GPIO_OUTPUT_Disable=56,
    ROM_API_INDEX_GLB_GPIO_Set_HZ=57,
    ROM_API_INDEX_GLB_GPIO_Get_Fun=58,
    ROM_API_INDEX_HBN_Mode_Enter=59,
    ROM_API_INDEX_HBN_Power_Down_Flash=60,
    ROM_API_INDEX_HBN_Enable=61,
    ROM_API_INDEX_HBN_Reset=62,
    ROM_API_INDEX_HBN_Set_Ldo11_Aon_Vout=63,
    ROM_API_INDEX_HBN_Set_Ldo11_Rt_Vout=64,
    ROM_API_INDEX_HBN_Set_Ldo11_Soc_Vout=65,
    ROM_API_INDEX_HBN_32K_Sel=66,
    ROM_API_INDEX_HBN_Set_ROOT_CLK_Sel=67,
    ROM_API_INDEX_HBN_Power_On_Xtal_32K=68,
    ROM_API_INDEX_HBN_Power_Off_Xtal_32K=69,
    ROM_API_INDEX_HBN_Power_On_RC32K=70,
    ROM_API_INDEX_HBN_Power_Off_RC32K=71,
    ROM_API_INDEX_HBN_Trim_RC32K=72,
    ROM_API_INDEX_HBN_Hw_Pu_Pd_Cfg=73,
    ROM_API_INDEX_HBN_Pin_WakeUp_Mask=74,
    ROM_API_INDEX_HBN_GPIO7_Dbg_Pull_Cfg=75,
    ROM_API_INDEX_HBN_Set_Embedded_Flash_Pullup=76,
    ROM_API_INDEX_L1C_Set_Wrap=77,
    ROM_API_INDEX_L1C_Set_Way_Disable=78,
    ROM_API_INDEX_L1C_IROM_2T_Access_Set=79,
    ROM_API_INDEX_PDS_Reset=80,
    ROM_API_INDEX_PDS_Enable=81,
    ROM_API_INDEX_PDS_Force_Config=82,
    ROM_API_INDEX_PDS_RAM_Config=83,
    ROM_API_INDEX_PDS_Default_Level_Config=84,
    ROM_API_INDEX_PDS_Trim_RC32M=85,
    ROM_API_INDEX_PDS_Select_RC32M_As_PLL_Ref=86,
    ROM_API_INDEX_PDS_Select_XTAL_As_PLL_Ref=87,
    ROM_API_INDEX_PDS_Power_On_PLL=88,
    ROM_API_INDEX_PDS_Enable_PLL_All_Clks=89,
    ROM_API_INDEX_PDS_Disable_PLL_All_Clks=90,
    ROM_API_INDEX_PDS_Enable_PLL_Clk=91,
    ROM_API_INDEX_PDS_Disable_PLL_Clk=92,
    ROM_API_INDEX_PDS_Power_Off_PLL=93,
    ROM_API_INDEX_SEC_Eng_Turn_On_Sec_Ring=94,
    ROM_API_INDEX_SEC_Eng_Turn_Off_Sec_Ring=95,
    ROM_API_INDEX_SFlash_Init=96,
    ROM_API_INDEX_SFlash_SetSPIMode=97,
    ROM_API_INDEX_SFlash_Read_Reg=98,
    ROM_API_INDEX_SFlash_Write_Reg=99,
    ROM_API_INDEX_SFlash_Busy=100,
    ROM_API_INDEX_SFlash_Write_Enable=101,
    ROM_API_INDEX_SFlash_Qspi_Enable=102,
    ROM_API_INDEX_SFlash_Volatile_Reg_Write_Enable=103,
    ROM_API_INDEX_SFlash_Chip_Erase=104,
    ROM_API_INDEX_SFlash_Sector_Erase=105,
    ROM_API_INDEX_SFlash_Blk32_Erase=106,
    ROM_API_INDEX_SFlash_Blk64_Erase=107,
    ROM_API_INDEX_SFlash_Erase=108,
    ROM_API_INDEX_SFlash_Program=109,
    ROM_API_INDEX_SFlash_GetUniqueId=110,
    ROM_API_INDEX_SFlash_GetJedecId=111,
    ROM_API_INDEX_SFlash_GetDeviceId=112,
    ROM_API_INDEX_SFlash_Powerdown=113,
    ROM_API_INDEX_SFlash_Releae_Powerdown=114,
    ROM_API_INDEX_SFlash_SetBurstWrap=115,
    ROM_API_INDEX_SFlash_DisableBurstWrap=116,
    ROM_API_INDEX_SFlash_Software_Reset=117,
    ROM_API_INDEX_SFlash_Reset_Continue_Read=118,
    ROM_API_INDEX_SFlash_Set_IDbus_Cfg=119,
    ROM_API_INDEX_SFlash_IDbus_Read_Enable=120,
    ROM_API_INDEX_SFlash_Cache_Enable_Set=121,
    ROM_API_INDEX_SFlash_Cache_Flush=122,
    ROM_API_INDEX_SFlash_Cache_Read_Enable=123,
    ROM_API_INDEX_SFlash_Cache_Hit_Count_Get=124,
    ROM_API_INDEX_SFlash_Cache_Miss_Count_Get=125,
    ROM_API_INDEX_SFlash_Cache_Read_Disable=126,
    ROM_API_INDEX_SFlash_Read=127,
    ROM_API_INDEX_FUNC_EMPTY_END=511
} anon_enum_16.conflict1b7af;

typedef struct tag_eflash_loader_if_cfg_t tag_eflash_loader_if_cfg_t, *Ptag_eflash_loader_if_cfg_t;

typedef int int32_t;

typedef struct tag_eflash_loader_if_cfg_t eflash_loader_if_cfg_t;

typedef int32_t (* boot_if_init_p)(void *);

typedef int32_t (* boot_if_handshake_poll_p)(void *);

struct tag_eflash_loader_if_cfg_t {
    uint8_t if_type;
    uint8_t if_type_onfail;
    uint8_t disabled;
    uint8_t rsvd;
    uint16_t maxlen;
    uint16_t timeout;
    boot_if_init_p boot_if_init;
    boot_if_handshake_poll_p boot_if_handshake_poll;
    uint32_t * (* boot_if_recv)(uint32_t *, uint32_t, uint32_t);
    int32_t (* boot_if_send)(uint32_t *, uint32_t);
    int32_t (* boot_if_wait_tx_idle)(uint32_t);
    int32_t (* boot_if_deinit)(void);
    int32_t (* boot_if_changerate)(uint32_t, uint32_t);
};

typedef enum tag_eflash_loader_if_type_t {
    BFLB_EFLASH_LOADER_IF_UART=0,
    BFLB_EFLASH_LOADER_IF_JLINK=1,
    BFLB_EFLASH_LOADER_IF_SDIO=2,
    BFLB_EFLASH_LOADER_IF_ALL=3
} tag_eflash_loader_if_type_t;

typedef enum tag_eflash_loader_if_type_t eflash_loader_if_type_t;

typedef enum SF_Ctrl_Sahb_Type {
    SF_CTRL_SAHB_CLOCK=0,
    SF_CTRL_FLASH_CLOCK=1
} SF_Ctrl_Sahb_Type;

typedef enum SF_Ctrl_Dmy_Mode_Type {
    SF_CTRL_DUMMY_1_LINE=0,
    SF_CTRL_DUMMY_2_LINES=1,
    SF_CTRL_DUMMY_4_LINES=2
} SF_Ctrl_Dmy_Mode_Type;

typedef enum SF_Ctrl_Mode_Type {
    SF_CTRL_SPI_MODE=0,
    SF_CTRL_QPI_MODE=1
} SF_Ctrl_Mode_Type;

typedef enum SF_Ctrl_IO_Type {
    SF_CTRL_NIO_MODE=0,
    SF_CTRL_DO_MODE=1,
    SF_CTRL_QO_MODE=2,
    SF_CTRL_DIO_MODE=3,
    SF_CTRL_QIO_MODE=4
} SF_Ctrl_IO_Type;

typedef enum SF_Ctrl_Data_Mode_Type {
    SF_CTRL_DATA_1_LINE=0,
    SF_CTRL_DATA_2_LINES=1,
    SF_CTRL_DATA_4_LINES=2
} SF_Ctrl_Data_Mode_Type;

typedef struct SF_Ctrl_Cmd_Cfg_Type SF_Ctrl_Cmd_Cfg_Type, *PSF_Ctrl_Cmd_Cfg_Type;

typedef enum SF_Ctrl_Cmd_Mode_Type {
    SF_CTRL_CMD_1_LINE=0,
    SF_CTRL_CMD_4_LINES=1
} SF_Ctrl_Cmd_Mode_Type;

typedef enum SF_Ctrl_Addr_Mode_Type {
    SF_CTRL_ADDR_1_LINE=0,
    SF_CTRL_ADDR_2_LINES=1,
    SF_CTRL_ADDR_4_LINES=2
} SF_Ctrl_Addr_Mode_Type;

struct SF_Ctrl_Cmd_Cfg_Type {
    uint8_t rwFlag;
    enum SF_Ctrl_Cmd_Mode_Type cmdMode;
    enum SF_Ctrl_Addr_Mode_Type addrMode;
    uint8_t addrSize;
    uint8_t dummyClks;
    enum SF_Ctrl_Dmy_Mode_Type dummyMode;
    enum SF_Ctrl_Data_Mode_Type dataMode;
    uint8_t rsv[1];
    uint32_t nbData;
    uint32_t cmdBuf[2];
};

typedef struct SF_Ctrl_Cfg_Type SF_Ctrl_Cfg_Type, *PSF_Ctrl_Cfg_Type;

typedef enum SF_Ctrl_Owner_Type {
    SF_CTRL_OWNER_SAHB=0,
    SF_CTRL_OWNER_IAHB=1
} SF_Ctrl_Owner_Type;

typedef enum SF_Ctrl_Ahb2sif_Type {
    HIGH_SPEED_MODE_CLOCK=0,
    REMOVE_CLOCK_CONSTRAIN=1
} SF_Ctrl_Ahb2sif_Type;

struct SF_Ctrl_Cfg_Type {
    enum SF_Ctrl_Owner_Type owner;
    enum SF_Ctrl_Sahb_Type sahbClock;
    enum SF_Ctrl_Ahb2sif_Type ahb2sifMode;
    uint8_t clkDelay;
    uint8_t clkInvert;
    uint8_t rxClkInvert;
    uint8_t doDelay;
    uint8_t diDelay;
    uint8_t oeDelay;
};

typedef enum SF_Ctrl_Pad_Sel {
    SF_CTRL_EMBEDDED_SEL=0,
    SF_CTRL_EXTERNAL_17TO22_SEL=1,
    SF_CTRL_EXTERNAL_0TO2_20TO22_SEL=2
} SF_Ctrl_Pad_Sel;

typedef enum SF_Ctrl_AES_Key_Type {
    SF_CTRL_AES_128BITS=0,
    SF_CTRL_AES_256BITS=1,
    SF_CTRL_AES_192BITS=2,
    SF_CTRL_AES_128BITS_DOUBLE_KEY=3
} SF_Ctrl_AES_Key_Type;

typedef enum GLB_DIG_CLK_Type {
    GLB_DIG_CLK_PLL_32M=0,
    GLB_DIG_CLK_XCLK=1
} GLB_DIG_CLK_Type;

typedef enum BMX_BUS_ERR_Type {
    BMX_BUS_ERR_TRUSTZONE_DECODE=0,
    BMX_BUS_ERR_ADDR_DECODE=1
} BMX_BUS_ERR_Type;

typedef enum GLB_SPI_PAD_ACT_AS_Type {
    GLB_SPI_PAD_ACT_AS_SLAVE=0,
    GLB_SPI_PAD_ACT_AS_MASTER=1
} GLB_SPI_PAD_ACT_AS_Type;

typedef enum GLB_EM_Type {
    GLB_EM_0KB=0,
    GLB_EM_8KB=3,
    GLB_EM_16KB=15
} GLB_EM_Type;

typedef enum BMX_ERR_INT_Type {
    BMX_ERR_INT_ERR=0,
    BMX_ERR_INT_ALL=1
} BMX_ERR_INT_Type;

typedef struct BMX_Cfg_Type BMX_Cfg_Type, *PBMX_Cfg_Type;

typedef enum BMX_ARB_Type {
    BMX_ARB_FIX=0,
    BMX_ARB_ROUND_ROBIN=1,
    BMX_ARB_RANDOM=2
} BMX_ARB_Type;

struct BMX_Cfg_Type {
    uint8_t timeoutEn;
    enum BL_Fun_Type errEn;
    enum BMX_ARB_Type arbMod;
};

typedef enum GLB_PLL_XTAL_Type {
    GLB_PLL_XTAL_NONE=0,
    GLB_PLL_XTAL_24M=1,
    GLB_PLL_XTAL_32M=2,
    GLB_PLL_XTAL_38P4M=3,
    GLB_PLL_XTAL_40M=4,
    GLB_PLL_XTAL_26M=5,
    GLB_PLL_XTAL_RC32M=6
} GLB_PLL_XTAL_Type;

typedef enum GLB_SYS_CLK_Type {
    GLB_SYS_CLK_RC32M=0,
    GLB_SYS_CLK_XTAL=1,
    GLB_SYS_CLK_PLL48M=2,
    GLB_SYS_CLK_PLL120M=3,
    GLB_SYS_CLK_PLL160M=4,
    GLB_SYS_CLK_PLL192M=5
} GLB_SYS_CLK_Type;

typedef enum GLB_BT_BANDWIDTH_Type {
    GLB_BT_BANDWIDTH_1M=0,
    GLB_BT_BANDWIDTH_2M=1
} GLB_BT_BANDWIDTH_Type;

typedef enum GLB_GPIO_INT_TRIG_Type {
    GLB_GPIO_INT_TRIG_NEG_PULSE=0,
    GLB_GPIO_INT_TRIG_POS_PULSE=1,
    GLB_GPIO_INT_TRIG_NEG_LEVEL=2,
    GLB_GPIO_INT_TRIG_POS_LEVEL=3
} GLB_GPIO_INT_TRIG_Type;

typedef enum GLB_DMA_CLK_ID_Type {
    GLB_DMA_CLK_DMA0_CH0=0,
    GLB_DMA_CLK_DMA0_CH1=1,
    GLB_DMA_CLK_DMA0_CH2=2,
    GLB_DMA_CLK_DMA0_CH3=3
} GLB_DMA_CLK_ID_Type;

typedef enum GLB_UART_SIG_FUN_Type {
    GLB_UART_SIG_FUN_UART0_RTS=0,
    GLB_UART_SIG_FUN_UART0_CTS=1,
    GLB_UART_SIG_FUN_UART0_TXD=2,
    GLB_UART_SIG_FUN_UART0_RXD=3,
    GLB_UART_SIG_FUN_UART1_RTS=4,
    GLB_UART_SIG_FUN_UART1_CTS=5,
    GLB_UART_SIG_FUN_UART1_TXD=6,
    GLB_UART_SIG_FUN_UART1_RXD=7
} GLB_UART_SIG_FUN_Type;

typedef enum GLB_ROOT_CLK_Type {
    GLB_ROOT_CLK_RC32M=0,
    GLB_ROOT_CLK_XTAL=1,
    GLB_ROOT_CLK_PLL=2
} GLB_ROOT_CLK_Type;

typedef enum GLB_SFLASH_CLK_Type {
    GLB_SFLASH_CLK_120M=0,
    GLB_SFLASH_CLK_XTAL=1,
    GLB_SFLASH_CLK_48M=2,
    GLB_SFLASH_CLK_80M=3,
    GLB_SFLASH_CLK_BCLK=4,
    GLB_SFLASH_CLK_96M=5
} GLB_SFLASH_CLK_Type;

typedef enum BMX_TO_INT_Type {
    BMX_TO_INT_TIMEOUT=0,
    BMX_TO_INT_ALL=1
} BMX_TO_INT_Type;

typedef enum GLB_DAC_CLK_Type {
    GLB_DAC_CLK_32M=0,
    GLB_DAC_CLK_XCLK=1
} GLB_DAC_CLK_Type;

typedef enum GLB_IR_CLK_SRC_Type {
    GLB_IR_CLK_SRC_XCLK=0
} GLB_IR_CLK_SRC_Type;

typedef enum GLB_ADC_CLK_Type {
    GLB_ADC_CLK_96M=0,
    GLB_ADC_CLK_XCLK=1
} GLB_ADC_CLK_Type;

typedef enum GLB_GPIO_INT_CONTROL_Type {
    GLB_GPIO_INT_CONTROL_SYNC=0,
    GLB_GPIO_INT_CONTROL_ASYNC=1
} GLB_GPIO_INT_CONTROL_Type;

typedef enum GLB_MTIMER_CLK_Type {
    GLB_MTIMER_CLK_BCLK=0,
    GLB_MTIMER_CLK_32K=1
} GLB_MTIMER_CLK_Type;

typedef enum GLB_GPIO_REAL_MODE_Type {
    GLB_GPIO_REAL_MODE_REG=0,
    GLB_GPIO_REAL_MODE_SDIO=1,
    GLB_GPIO_REAL_MODE_RF=12,
    GLB_GPIO_REAL_MODE_JTAG=14,
    GLB_GPIO_REAL_MODE_CCI=15
} GLB_GPIO_REAL_MODE_Type;

typedef enum GLB_PKA_CLK_Type {
    GLB_PKA_CLK_HCLK=0,
    GLB_PKA_CLK_PLL120M=1
} GLB_PKA_CLK_Type;

typedef enum GLB_UART_SIG_Type {
    GLB_UART_SIG_0=0,
    GLB_UART_SIG_1=1,
    GLB_UART_SIG_2=2,
    GLB_UART_SIG_3=3,
    GLB_UART_SIG_4=4,
    GLB_UART_SIG_5=5,
    GLB_UART_SIG_6=6,
    GLB_UART_SIG_7=7
} GLB_UART_SIG_Type;

typedef struct eflash_loader_cmd_cfg_t eflash_loader_cmd_cfg_t, *Peflash_loader_cmd_cfg_t;

struct eflash_loader_cmd_cfg_t {
    uint8_t cmd;
    uint8_t enabled;
    undefined field2_0x2;
    undefined field3_0x3;
    int32_t (* cmd_process)(uint16_t, uint8_t *, uint16_t);
};

typedef ulonglong UDItype;

typedef uchar UQItype;

typedef uint USItype;

typedef struct DWstruct DWstruct, *PDWstruct;

typedef int SItype;

struct DWstruct {
    SItype low;
    SItype high;
};

typedef longlong DItype;

typedef union DWunion DWunion, *PDWunion;

union DWunion {
    struct DWstruct s;
    DItype ll;
};

typedef enum tag_eflash_loader_error_code_t {
    BFLB_EFLASH_LOADER_PLL_ERROR=-4,
    BFLB_EFLASH_LOADER_INVASION_ERROR=-3,
    BFLB_EFLASH_LOADER_POLLING=-2,
    BFLB_EFLASH_LOADER_FAIL=-1,
    BFLB_EFLASH_LOADER_SUCCESS=0,
    BFLB_EFLASH_LOADER_FLASH_INIT_ERROR=1,
    BFLB_EFLASH_LOADER_FLASH_ERASE_PARA_ERROR=2,
    BFLB_EFLASH_LOADER_FLASH_ERASE_ERROR=3,
    BFLB_EFLASH_LOADER_FLASH_WRITE_PARA_ERROR=4,
    BFLB_EFLASH_LOADER_FLASH_WRITE_ADDR_ERROR=5,
    BFLB_EFLASH_LOADER_FLASH_WRITE_ERROR=6,
    BFLB_EFLASH_LOADER_FLASH_BOOT_PARA_ERROR=7,
    BFLB_EFLASH_LOADER_FLASH_SET_PARA_ERROR=8,
    BFLB_EFLASH_LOADER_FLASH_READ_STATUS_REG_ERROR=9,
    BFLB_EFLASH_LOADER_FLASH_WRITE_STATUS_REG_ERROR=10,
    BFLB_EFLASH_LOADER_FLASH_DECOMPRESS_WRITE_ERROR=11,
    BFLB_EFLASH_LOADER_FLASH_WRITE_XZ_ERROR=12,
    BFLB_EFLASH_LOADER_CMD_ID_ERROR=257,
    BFLB_EFLASH_LOADER_CMD_LEN_ERROR=258,
    BFLB_EFLASH_LOADER_CMD_CRC_ERROR=259,
    BFLB_EFLASH_LOADER_CMD_SEQ_ERROR=260,
    BFLB_EFLASH_LOADER_IMG_BOOTHEADER_LEN_ERROR=513,
    BFLB_EFLASH_LOADER_IMG_BOOTHEADER_NOT_LOAD_ERROR=514,
    BFLB_EFLASH_LOADER_IMG_BOOTHEADER_MAGIC_ERROR=515,
    BFLB_EFLASH_LOADER_IMG_BOOTHEADER_CRC_ERROR=516,
    BFLB_EFLASH_LOADER_IMG_BOOTHEADER_ENCRYPT_NOTFIT=517,
    BFLB_EFLASH_LOADER_IMG_BOOTHEADER_SIGN_NOTFIT=518,
    BFLB_EFLASH_LOADER_IMG_SEGMENT_CNT_ERROR=519,
    BFLB_EFLASH_LOADER_IMG_AES_IV_LEN_ERROR=520,
    BFLB_EFLASH_LOADER_IMG_AES_IV_CRC_ERROR=521,
    BFLB_EFLASH_LOADER_IMG_PK_LEN_ERROR=522,
    BFLB_EFLASH_LOADER_IMG_PK_CRC_ERROR=523,
    BFLB_EFLASH_LOADER_IMG_PK_HASH_ERROR=524,
    BFLB_EFLASH_LOADER_IMG_SIGNATURE_LEN_ERROR=525,
    BFLB_EFLASH_LOADER_IMG_SIGNATURE_CRC_ERROR=526,
    BFLB_EFLASH_LOADER_IMG_SECTIONHEADER_LEN_ERROR=527,
    BFLB_EFLASH_LOADER_IMG_SECTIONHEADER_CRC_ERROR=528,
    BFLB_EFLASH_LOADER_IMG_SECTIONHEADER_DST_ERROR=529,
    BFLB_EFLASH_LOADER_IMG_SECTIONDATA_LEN_ERROR=530,
    BFLB_EFLASH_LOADER_IMG_SECTIONDATA_DEC_ERROR=531,
    BFLB_EFLASH_LOADER_IMG_SECTIONDATA_TLEN_ERROR=532,
    BFLB_EFLASH_LOADER_IMG_SECTIONDATA_CRC_ERROR=533,
    BFLB_EFLASH_LOADER_IMG_HALFBAKED_ERROR=534,
    BFLB_EFLASH_LOADER_IMG_HASH_ERROR=535,
    BFLB_EFLASH_LOADER_IMG_SIGN_PARSE_ERROR=536,
    BFLB_EFLASH_LOADER_IMG_SIGN_ERROR=537,
    BFLB_EFLASH_LOADER_IMG_DEC_ERROR=538,
    BFLB_EFLASH_LOADER_IMG_ALL_INVALID_ERROR=539,
    BFLB_EFLASH_LOADER_IF_RATE_LEN_ERROR=769,
    BFLB_EFLASH_LOADER_IF_RATE_PARA_ERROR=770,
    BFLB_EFLASH_LOADER_IF_PASSWORDERROR=771,
    BFLB_EFLASH_LOADER_IF_PASSWORDCLOSE=772,
    BFLB_EFLASH_LOADER_EFUSE_WRITE_PARA_ERROR=1025,
    BFLB_EFLASH_LOADER_EFUSE_WRITE_ADDR_ERROR=1026,
    BFLB_EFLASH_LOADER_EFUSE_WRITE_ERROR=1027,
    BFLB_EFLASH_LOADER_EFUSE_READ_PARA_ERROR=1028,
    BFLB_EFLASH_LOADER_EFUSE_READ_ADDR_ERROR=1029,
    BFLB_EFLASH_LOADER_EFUSE_READ_ERROR=1030,
    BFLB_EFLASH_LOADER_EFUSE_READ_MAC_ERROR=1031
} tag_eflash_loader_error_code_t;

typedef struct boot_efuse_sw_cfg0_t boot_efuse_sw_cfg0_t, *Pboot_efuse_sw_cfg0_t;

struct boot_efuse_sw_cfg0_t {
    uint32_t bootrom_protect:1;
    uint32_t uart_log_disable:1;
    uint32_t sdio_pin_init:1;
    uint32_t cci_coexist_disable:1;
    uint32_t xtal_type:3;
    uint32_t pll_clk:3;
    uint32_t hclk_div:1;
    uint32_t bclk_div:1;
    uint32_t flash_clk_type:3;
    uint32_t flash_clk_div:1;
    uint32_t flash_cfg:2;
    uint32_t flash_power_delay:2;
    uint32_t tz_boot:1;
    uint32_t encrypted_tz_boot:1;
    uint32_t hbn_check_sign:1;
    uint32_t keep_dbg_port_closed:1;
    uint32_t mediaboot_disable:1;
    uint32_t uartboot_disable:1;
    uint32_t sdioboot_disable:1;
    uint32_t hbn_jump_disable:1;
    uint32_t jtag_switch:1;
    uint32_t jtag_init:1;
    uint32_t qfn40:1;
    uint32_t debug_log_reopen:1;
};

typedef struct boot_efuse_hw_cfg_t boot_efuse_hw_cfg_t, *Pboot_efuse_hw_cfg_t;

struct boot_efuse_hw_cfg_t {
    uint8_t sign_type[1];
    uint8_t encrypted[1];
    uint8_t soft_dbg_config;
    uint8_t trim_enable;
};

typedef struct boot_otp_cfg_t boot_otp_cfg_t, *Pboot_otp_cfg_t;

struct boot_otp_cfg_t {
    struct boot_efuse_hw_cfg_t hw_cfg;
    struct boot_efuse_sw_cfg0_t sw_cfg0;
    uint8_t chip_id[8];
};

typedef enum BL_AHB_Slave1_Type {
    BL_AHB_SLAVE1_GLB=0,
    BL_AHB_SLAVE1_RF=1,
    BL_AHB_SLAVE1_GPIP_PHY_AGC=2,
    BL_AHB_SLAVE1_SEC_DBG=3,
    BL_AHB_SLAVE1_SEC=4,
    BL_AHB_SLAVE1_TZ1=5,
    BL_AHB_SLAVE1_TZ2=6,
    BL_AHB_SLAVE1_EFUSE=7,
    BL_AHB_SLAVE1_CCI=8,
    BL_AHB_SLAVE1_L1C=9,
    BL_AHB_SLAVE1_RSVD0A=10,
    BL_AHB_SLAVE1_SFC=11,
    BL_AHB_SLAVE1_DMA=12,
    BL_AHB_SLAVE1_SDU=13,
    BL_AHB_SLAVE1_PDS_HBN_AON_HBNRAM=14,
    BL_AHB_SLAVE1_RSVD0F=15,
    BL_AHB_SLAVE1_UART0=16,
    BL_AHB_SLAVE1_UART1=17,
    BL_AHB_SLAVE1_SPI=18,
    BL_AHB_SLAVE1_I2C=19,
    BL_AHB_SLAVE1_PWM=20,
    BL_AHB_SLAVE1_TMR=21,
    BL_AHB_SLAVE1_IRR=22,
    BL_AHB_SLAVE1_CKS=23,
    BL_AHB_SLAVE1_MAX=24
} BL_AHB_Slave1_Type;

typedef enum IRQn_Type {
    MSOFT_IRQn=3,
    MTIME_IRQn=7,
    MEXT_IRQn=11,
    CLIC_SOFT_PEND_IRQn=12,
    BMX_ERR_IRQn=16,
    BMX_TO_IRQn=17,
    L1C_BMX_ERR_IRQn=18,
    L1C_BMX_TO_IRQn=19,
    SEC_BMX_ERR_IRQn=20,
    RF_TOP_INT0_IRQn=21,
    RF_TOP_INT1_IRQn=22,
    SDIO_IRQn=23,
    DMA_BMX_ERR_IRQn=24,
    SEC_GMAC_IRQn=25,
    SEC_CDET_IRQn=26,
    SEC_PKA_IRQn=27,
    SEC_TRNG_IRQn=28,
    SEC_AES_IRQn=29,
    SEC_SHA_IRQn=30,
    DMA_ALL_IRQn=31,
    RESERVED0=32,
    RESERVED1=33,
    RESERVED2=34,
    IRTX_IRQn=35,
    IRRX_IRQn=36,
    RESERVED3=37,
    RESERVED4=38,
    SF_CTRL_IRQn=39,
    RESERVED5=40,
    GPADC_DMA_IRQn=41,
    EFUSE_IRQn=42,
    SPI_IRQn=43,
    RESERVED6=44,
    UART0_IRQn=45,
    UART1_IRQn=46,
    RESERVED7=47,
    I2C_IRQn=48,
    RESERVED8=49,
    PWM_IRQn=50,
    RESERVED9=51,
    TIMER_CH0_IRQn=52,
    TIMER_CH1_IRQn=53,
    TIMER_WDT_IRQn=54,
    RESERVED10=55,
    RESERVED11=56,
    RESERVED12=57,
    RESERVED13=58,
    RESERVED14=59,
    GPIO_INT0_IRQn=60,
    RESERVED16=61,
    RESERVED17=62,
    RESERVED18=63,
    RESERVED19=64,
    RESERVED20=65,
    PDS_WAKEUP_IRQn=66,
    HBN_OUT0_IRQn=67,
    HBN_OUT1_IRQn=68,
    BOR_IRQn=69,
    WIFI_IRQn=70,
    BZ_PHY_IRQn=71,
    BLE_IRQn=72,
    MAC_TXRX_TIMER_IRQn=73,
    MAC_TXRX_MISC_IRQn=74,
    MAC_RX_TRG_IRQn=75,
    MAC_TX_TRG_IRQn=76,
    MAC_GEN_IRQn=77,
    MAC_PORT_TRG_IRQn=78,
    WIFI_IPC_PUBLIC_IRQn=79,
    IRQn_LAST=80
} IRQn_Type;

typedef struct anon_struct.conflictbb9d anon_struct.conflictbb9d, *Panon_struct.conflictbb9d;

struct anon_struct.conflictbb9d {
    uint32_t size;
    uint8_t buf[63];
    undefined field2_0x43;
};

typedef struct SEC_Eng_SHA256_Link_Ctx SEC_Eng_SHA256_Link_Ctx, *PSEC_Eng_SHA256_Link_Ctx;

struct SEC_Eng_SHA256_Link_Ctx {
    uint32_t total[2];
    uint32_t * shaBuf;
    uint32_t * shaPadding;
    uint32_t linkAddr;
};

typedef enum SEC_ENG_SHA_ID_Type {
    SEC_ENG_SHA_ID0=0
} SEC_ENG_SHA_ID_Type;

typedef enum SEC_ENG_AES_ID_Type {
    SEC_ENG_AES_ID0=0
} SEC_ENG_AES_ID_Type;

typedef enum SEC_ENG_AES_Key_Type {
    SEC_ENG_AES_KEY_128BITS=0,
    SEC_ENG_AES_KEY_256BITS=1,
    SEC_ENG_AES_KEY_192BITS=2,
    SEC_ENG_AES_DOUBLE_KEY_128BITS=3
} SEC_ENG_AES_Key_Type;

typedef enum SEC_ENG_AES_Counter_Type {
    SEC_ENG_AES_COUNTER_BYTE_4=0,
    SEC_ENG_AES_COUNTER_BYTE_1=1,
    SEC_ENG_AES_COUNTER_BYTE_2=2,
    SEC_ENG_AES_COUNTER_BYTE_3=3
} SEC_ENG_AES_Counter_Type;

typedef enum SEC_ENG_PKA_REG_SIZE_Type {
    SEC_ENG_PKA_REG_SIZE_8=1,
    SEC_ENG_PKA_REG_SIZE_16=2,
    SEC_ENG_PKA_REG_SIZE_32=3,
    SEC_ENG_PKA_REG_SIZE_64=4,
    SEC_ENG_PKA_REG_SIZE_96=5,
    SEC_ENG_PKA_REG_SIZE_128=6,
    SEC_ENG_PKA_REG_SIZE_192=7,
    SEC_ENG_PKA_REG_SIZE_256=8,
    SEC_ENG_PKA_REG_SIZE_384=9,
    SEC_ENG_PKA_REG_SIZE_512=10
} SEC_ENG_PKA_REG_SIZE_Type;

typedef struct SEC_Eng_SHA256_Ctx SEC_Eng_SHA256_Ctx, *PSEC_Eng_SHA256_Ctx;

struct SEC_Eng_SHA256_Ctx {
    uint32_t total[2];
    uint32_t * shaBuf;
    uint32_t * shaPadding;
    uint8_t shaFeed;
    undefined field4_0x11;
    undefined field5_0x12;
    undefined field6_0x13;
};

typedef enum SEC_ENG_SHA_Type {
    SEC_ENG_SHA256=0,
    SEC_ENG_SHA224=1,
    SEC_ENG_SHA1=2,
    SEC_ENG_SHA1_RSVD=3
} SEC_ENG_SHA_Type;

typedef enum SEC_ENG_PKA_OP_Type {
    SEC_ENG_PKA_OP_PPSEL=0,
    SEC_ENG_PKA_OP_MOD2N=17,
    SEC_ENG_PKA_OP_LDIV2N=18,
    SEC_ENG_PKA_OP_LMUL2N=19,
    SEC_ENG_PKA_OP_LDIV=20,
    SEC_ENG_PKA_OP_LSQR=21,
    SEC_ENG_PKA_OP_LMUL=22,
    SEC_ENG_PKA_OP_LSUB=23,
    SEC_ENG_PKA_OP_LADD=24,
    SEC_ENG_PKA_OP_LCMP=25,
    SEC_ENG_PKA_OP_MDIV2=33,
    SEC_ENG_PKA_OP_MINV=34,
    SEC_ENG_PKA_OP_MEXP=35,
    SEC_ENG_PKA_OP_MSQR=36,
    SEC_ENG_PKA_OP_MMUL=37,
    SEC_ENG_PKA_OP_MREM=38,
    SEC_ENG_PKA_OP_MSUB=39,
    SEC_ENG_PKA_OP_MADD=40,
    SEC_ENG_PKA_OP_RESIZE=49,
    SEC_ENG_PKA_OP_MOVDAT=50,
    SEC_ENG_PKA_OP_NLIR=51,
    SEC_ENG_PKA_OP_SLIR=52,
    SEC_ENG_PKA_OP_CLIR=53,
    SEC_ENG_PKA_OP_CFLIRI_BUFFER=54,
    SEC_ENG_PKA_OP_CTLIRI_PLD=55,
    SEC_ENG_PKA_OP_CFLIR_BUFFER=56,
    SEC_ENG_PKA_OP_CTLIR_PLD=57
} SEC_ENG_PKA_OP_Type;

typedef enum SEC_ENG_AES_Key_Src_Type {
    SEC_ENG_AES_KEY_SW=0,
    SEC_ENG_AES_KEY_HW=1
} SEC_ENG_AES_Key_Src_Type;

typedef struct SEC_Eng_AES_Ctx SEC_Eng_AES_Ctx, *PSEC_Eng_AES_Ctx;

typedef enum SEC_ENG_AES_Type {
    SEC_ENG_AES_ECB=0,
    SEC_ENG_AES_CTR=1,
    SEC_ENG_AES_CBC=2
} SEC_ENG_AES_Type;

struct SEC_Eng_AES_Ctx {
    uint8_t aesFeed;
    enum SEC_ENG_AES_Type mode;
};

typedef enum SEC_ENG_INT_Type {
    SEC_ENG_INT_TRNG=0,
    SEC_ENG_INT_AES=1,
    SEC_ENG_INT_SHA=2,
    SEC_ENG_INT_PKA=3,
    SEC_ENG_INT_CDET=4,
    SEC_ENG_INT_GMAC=5,
    SEC_ENG_INT_ALL=6
} SEC_ENG_INT_Type;

typedef enum SEC_ENG_AES_EnDec_Type {
    SEC_ENG_AES_ENCRYPTION=0,
    SEC_ENG_AES_DECRYPTION=1
} SEC_ENG_AES_EnDec_Type;

typedef struct Flash_Info_t Flash_Info_t, *PFlash_Info_t;

struct Flash_Info_t {
    uint32_t jedecID;
    char * name;
    struct SPI_Flash_Cfg_Type * cfg;
};

typedef struct pka0_bit_shift_op_cfg pka0_bit_shift_op_cfg, *Ppka0_bit_shift_op_cfg;

typedef union anon_union.conflict16291_for_value anon_union.conflict16291_for_value, *Panon_union.conflict16291_for_value;

typedef struct anon_struct.conflict16267 anon_struct.conflict16267, *Panon_struct.conflict16267;

struct anon_struct.conflict16267 {
    uint32_t bit_shift:15;
    uint32_t reserved_24_31:17;
};

union anon_union.conflict16291_for_value {
    struct anon_struct.conflict16267 BF;
    uint32_t WORD;
};

struct pka0_bit_shift_op_cfg {
    union anon_union.conflict16291_for_value value;
};

typedef struct pka0_common_op_first_cfg pka0_common_op_first_cfg, *Ppka0_common_op_first_cfg;

typedef union anon_union.conflict16099_for_value anon_union.conflict16099_for_value, *Panon_union.conflict16099_for_value;

typedef struct anon_struct.conflict16030 anon_struct.conflict16030, *Panon_struct.conflict16030;

struct anon_struct.conflict16030 {
    uint32_t s0_reg_idx:8;
    uint32_t s0_reg_type:4;
    uint32_t d_reg_idx:8;
    uint32_t d_reg_type:4;
    uint32_t op:7;
    uint32_t last_op:1;
};

union anon_union.conflict16099_for_value {
    struct anon_struct.conflict16030 BF;
    uint32_t WORD;
};

struct pka0_common_op_first_cfg {
    union anon_union.conflict16099_for_value value;
};

typedef union anon_union.conflict1611f anon_union.conflict1611f, *Panon_union.conflict1611f;

typedef struct anon_struct.conflict160d5 anon_struct.conflict160d5, *Panon_struct.conflict160d5;

struct anon_struct.conflict160d5 {
    uint32_t reserved_0_11:12;
    uint32_t s1_reg_idx:8;
    uint32_t s1_reg_type:4;
    uint32_t reserved_24_31:8;
};

union anon_union.conflict1611f {
    struct anon_struct.conflict160d5 BF;
    uint32_t WORD;
};

typedef struct anon_struct.conflict1615b anon_struct.conflict1615b, *Panon_struct.conflict1615b;

struct anon_struct.conflict1615b {
    uint32_t s2_reg_idx:8;
    uint32_t s2_reg_type:4;
    uint32_t reserved_12_31:20;
};

typedef struct pka0_pld_cfg pka0_pld_cfg, *Ppka0_pld_cfg;

typedef union anon_union.conflict15f5f_for_value anon_union.conflict15f5f_for_value, *Panon_union.conflict15f5f_for_value;

typedef struct anon_struct.conflict15f06 anon_struct.conflict15f06, *Panon_struct.conflict15f06;

struct anon_struct.conflict15f06 {
    uint32_t size:12;
    uint32_t d_reg_index:8;
    uint32_t d_reg_type:4;
    uint32_t op:7;
    uint32_t last_op:1;
};

union anon_union.conflict15f5f_for_value {
    struct anon_struct.conflict15f06 BF;
    uint32_t WORD;
};

struct pka0_pld_cfg {
    union anon_union.conflict15f5f_for_value value;
};

typedef struct pka0_pldi_cfg pka0_pldi_cfg, *Ppka0_pldi_cfg;

typedef union anon_union.conflict15ff4_for_value anon_union.conflict15ff4_for_value, *Panon_union.conflict15ff4_for_value;

typedef struct anon_struct.conflict15f9b anon_struct.conflict15f9b, *Panon_struct.conflict15f9b;

struct anon_struct.conflict15f9b {
    uint32_t rsvd:12;
    uint32_t d_reg_index:8;
    uint32_t d_reg_type:4;
    uint32_t op:7;
    uint32_t last_op:1;
};

union anon_union.conflict15ff4_for_value {
    struct anon_struct.conflict15f9b BF;
    uint32_t WORD;
};

struct pka0_pldi_cfg {
    union anon_union.conflict15ff4_for_value value;
};

typedef struct pka0_common_op_snd_cfg_S2_only pka0_common_op_snd_cfg_S2_only, *Ppka0_common_op_snd_cfg_S2_only;

typedef union anon_union.conflict16195_for_value anon_union.conflict16195_for_value, *Panon_union.conflict16195_for_value;

union anon_union.conflict16195_for_value {
    struct anon_struct.conflict1615b BF;
    uint32_t WORD;
};

struct pka0_common_op_snd_cfg_S2_only {
    union anon_union.conflict16195_for_value value;
};

typedef union anon_union.conflict1622b anon_union.conflict1622b, *Panon_union.conflict1622b;

typedef struct anon_struct.conflict161d1 anon_struct.conflict161d1, *Panon_struct.conflict161d1;

struct anon_struct.conflict161d1 {
    uint32_t s2_reg_idx:8;
    uint32_t s2_reg_type:4;
    uint32_t s1_reg_idx:8;
    uint32_t s1_reg_type:4;
    uint32_t reserved_24_31:8;
};

union anon_union.conflict1622b {
    struct anon_struct.conflict161d1 BF;
    uint32_t WORD;
};

typedef union anon_union.conflict16099 anon_union.conflict16099, *Panon_union.conflict16099;

union anon_union.conflict16099 {
    struct anon_struct.conflict16030 BF;
    uint32_t WORD;
};

typedef union anon_union.conflict15f5f anon_union.conflict15f5f, *Panon_union.conflict15f5f;

union anon_union.conflict15f5f {
    struct anon_struct.conflict15f06 BF;
    uint32_t WORD;
};

typedef struct pka0_common_op_snd_cfg_S1_only pka0_common_op_snd_cfg_S1_only, *Ppka0_common_op_snd_cfg_S1_only;

typedef union anon_union.conflict1611f_for_value anon_union.conflict1611f_for_value, *Panon_union.conflict1611f_for_value;

union anon_union.conflict1611f_for_value {
    struct anon_struct.conflict160d5 BF;
    uint32_t WORD;
};

struct pka0_common_op_snd_cfg_S1_only {
    union anon_union.conflict1611f_for_value value;
};

typedef union anon_union.conflict16291 anon_union.conflict16291, *Panon_union.conflict16291;

union anon_union.conflict16291 {
    struct anon_struct.conflict16267 BF;
    uint32_t WORD;
};

typedef struct pka0_common_op_snd_cfg_S1_S2 pka0_common_op_snd_cfg_S1_S2, *Ppka0_common_op_snd_cfg_S1_S2;

typedef union anon_union.conflict1622b_for_value anon_union.conflict1622b_for_value, *Panon_union.conflict1622b_for_value;

union anon_union.conflict1622b_for_value {
    struct anon_struct.conflict161d1 BF;
    uint32_t WORD;
};

struct pka0_common_op_snd_cfg_S1_S2 {
    union anon_union.conflict1622b_for_value value;
};

typedef union anon_union.conflict16195 anon_union.conflict16195, *Panon_union.conflict16195;

union anon_union.conflict16195 {
    struct anon_struct.conflict1615b BF;
    uint32_t WORD;
};

typedef union anon_union.conflict15ff4 anon_union.conflict15ff4, *Panon_union.conflict15ff4;

union anon_union.conflict15ff4 {
    struct anon_struct.conflict15f9b BF;
    uint32_t WORD;
};

typedef char int8_t;

typedef longlong int64_t;

typedef uint64_t uintmax_t;

typedef int64_t intmax_t;

typedef uint uintptr_t;

typedef enum Elf_ProgramHeaderType_RISCV {
    PT_NULL=0,
    PT_LOAD=1,
    PT_DYNAMIC=2,
    PT_INTERP=3,
    PT_NOTE=4,
    PT_SHLIB=5,
    PT_PHDR=6,
    PT_TLS=7,
    PT_GNU_EH_FRAME=1685382480,
    PT_GNU_STACK=1685382481,
    PT_GNU_RELRO=1685382482
} Elf_ProgramHeaderType_RISCV;

typedef enum Elf_SectionHeaderType_RISCV {
    SHT_NULL=0,
    SHT_PROGBITS=1,
    SHT_SYMTAB=2,
    SHT_STRTAB=3,
    SHT_RELA=4,
    SHT_HASH=5,
    SHT_DYNAMIC=6,
    SHT_NOTE=7,
    SHT_NOBITS=8,
    SHT_REL=9,
    SHT_SHLIB=10,
    SHT_DYNSYM=11,
    SHT_INIT_ARRAY=14,
    SHT_FINI_ARRAY=15,
    SHT_PREINIT_ARRAY=16,
    SHT_GROUP=17,
    SHT_SYMTAB_SHNDX=18,
    SHT_ANDROID_REL=1610612737,
    SHT_ANDROID_RELA=1610612738,
    SHT_GNU_ATTRIBUTES=1879048181,
    SHT_GNU_HASH=1879048182,
    SHT_GNU_LIBLIST=1879048183,
    SHT_CHECKSUM=1879048184,
    SHT_SUNW_move=1879048186,
    SHT_SUNW_COMDAT=1879048187,
    SHT_SUNW_syminfo=1879048188,
    SHT_GNU_verdef=1879048189,
    SHT_GNU_verneed=1879048190,
    SHT_GNU_versym=1879048191
} Elf_SectionHeaderType_RISCV;

typedef struct Elf32_Shdr Elf32_Shdr, *PElf32_Shdr;

struct Elf32_Shdr {
    dword sh_name;
    enum Elf_SectionHeaderType_RISCV sh_type;
    dword sh_flags;
    dword sh_addr;
    dword sh_offset;
    dword sh_size;
    dword sh_link;
    dword sh_info;
    dword sh_addralign;
    dword sh_entsize;
};

typedef struct Elf32_Sym Elf32_Sym, *PElf32_Sym;

struct Elf32_Sym {
    dword st_name;
    dword st_value;
    dword st_size;
    byte st_info;
    byte st_other;
    word st_shndx;
};

typedef struct Elf32_Phdr Elf32_Phdr, *PElf32_Phdr;

struct Elf32_Phdr {
    enum Elf_ProgramHeaderType_RISCV p_type;
    dword p_offset;
    dword p_vaddr;
    dword p_paddr;
    dword p_filesz;
    dword p_memsz;
    dword p_flags;
    dword p_align;
};

typedef struct Elf32_Ehdr Elf32_Ehdr, *PElf32_Ehdr;

struct Elf32_Ehdr {
    byte e_ident_magic_num;
    char e_ident_magic_str[3];
    byte e_ident_class;
    byte e_ident_data;
    byte e_ident_version;
    byte e_ident_osabi;
    byte e_ident_abiversion;
    byte e_ident_pad[7];
    word e_type;
    word e_machine;
    dword e_version;
    dword e_entry;
    dword e_phoff;
    dword e_shoff;
    dword e_flags;
    word e_ehsize;
    word e_phentsize;
    word e_phnum;
    word e_shentsize;
    word e_shnum;
    word e_shstrndx;
};




// WARNING: Removing unreachable block (ram,0x2201003c)
// WARNING: Removing unreachable block (ram,0x22010024)
// WARNING: Removing unreachable block (ram,0x2201005a)
// WARNING: Removing unreachable block (ram,0x22010072)

void entry(void)

{
  SystemInit();
  start_load();
  main();
  do {
                    // WARNING: Do nothing block with infinite loop
  } while( true );
}



void FUN_22010074(void)

{
  do {
                    // WARNING: Do nothing block with infinite loop
  } while( true );
}



void * memmove(void *__dest,void *__src,size_t __n)

{
  undefined *puVar1;
  undefined uVar2;
  int iVar3;
  undefined *puVar4;
  undefined4 *puVar5;
  undefined4 *puVar6;
  void *pvVar7;
  undefined4 *puVar8;
  undefined4 *puVar9;
  uint uVar10;
  
  if ((__src < __dest) && (pvVar7 = (void *)((int)__src + __n), __dest < pvVar7)) {
    puVar4 = (undefined *)((int)__dest + __n);
    if (__n == 0) {
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return __dest;
    }
    do {
      puVar1 = (undefined *)((int)pvVar7 + -1);
      pvVar7 = (void *)((int)pvVar7 + -1);
      puVar4 = puVar4 + -1;
      *puVar4 = *puVar1;
    } while (__src != pvVar7);
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return __dest;
  }
  uVar10 = __n;
  puVar8 = (undefined4 *)__dest;
  if (0xf < __n) {
    if ((((uint)__src | (uint)__dest) & 3) != 0) {
      iVar3 = __n - 1;
      goto LAB_22010234;
    }
    iVar3 = (__n - 0x10 & 0xfffffff0) + 0x10;
    puVar5 = (undefined4 *)__src;
    puVar9 = (undefined4 *)__dest;
    do {
      puVar8 = puVar9 + 4;
      *puVar9 = *puVar5;
      puVar9[1] = puVar5[1];
      puVar9[2] = puVar5[2];
      puVar9[3] = puVar5[3];
      puVar5 = puVar5 + 4;
      puVar9 = puVar8;
    } while ((undefined4 *)((int)__dest + iVar3) != puVar8);
    __src = (void *)((int)__src + iVar3);
    uVar10 = __n & 0xf;
    puVar5 = (undefined4 *)__src;
    if ((__n & 0xc) != 0) {
      do {
        puVar6 = puVar5 + 1;
        *puVar9 = *puVar5;
        puVar5 = puVar6;
        puVar9 = puVar9 + 1;
      } while ((undefined *)0x3 < (undefined *)((uVar10 - (int)puVar6) + (int)__src));
      iVar3 = (uVar10 - 4 & 0xfffffffc) + 4;
      __src = (void *)((int)__src + iVar3);
      uVar10 = __n & 3;
      puVar8 = (undefined4 *)((int)puVar8 + iVar3);
    }
  }
  iVar3 = uVar10 - 1;
  if (uVar10 == 0) {
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return __dest;
  }
LAB_22010234:
  puVar5 = puVar8;
  do {
                    // WARNING: Load size is inaccurate
    uVar2 = *__src;
    puVar9 = (undefined4 *)((int)puVar5 + 1);
    __src = (void *)((int)__src + 1);
    *(undefined *)puVar5 = uVar2;
    puVar5 = puVar9;
  } while (puVar9 != (undefined4 *)(iVar3 + 1 + (int)puVar8));
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return __dest;
}



UDItype __udivdi3(UDItype n,UDItype d)

{
  UDItype in_fa0;
  
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return in_fa0;
}



UDItype __umoddi3(UDItype u,UDItype v)

{
  UDItype in_fa0;
  
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return in_fa0;
}



// WARNING: Variable defined which should be unmapped: sfCtrlCfg
// WARNING: Removing unreachable block (ram,0x22010b5c)
// WARNING: Removing unreachable block (ram,0x220109b4)
// WARNING: Removing unreachable block (ram,0x220109a8)
// WARNING: Removing unreachable block (ram,0x220109ae)
// WARNING: Removing unreachable block (ram,0x22010b4c)
// WARNING: Removing unreachable block (ram,0x22010b6c)
// WARNING: Could not reconcile some variable overlaps
// WARNING: Globals starting with '_' overlap smaller symbols at the same address

int main(void)

{
  uint uVar1;
  BL_Err_Type BVar2;
  undefined3 extraout_var;
  uint local_20;
  uint32_t swCfg;
  SF_Ctrl_Cfg_Type sfCtrlCfg;
  
  sfCtrlCfg._0_4_ = 0x101;
  local_20 = 0;
  swCfg = 0;
  sfCtrlCfg.clkInvert = '\0';
  xtalType = GLB_PLL_XTAL_40M;
  xtal_clk = 40000000;
  bootMode = bootrom_read_boot_mode();
  if (xtalType == GLB_PLL_XTAL_RC32M) {
    HBN_Set_ROOT_CLK_Sel(HBN_ROOT_CLK_RC32M);
  }
  else {
    HBN_Set_ROOT_CLK_Sel(HBN_ROOT_CLK_XTAL);
  }
  _DAT_4000f108 = xtal_clk;
  bflb_read_otp(&otp_cfg);
  EF_Ctrl_Read_Sw_Usage(0,&local_20);
  uVar1 = local_20 >> 0x10;
  bflb_set_high_speed_system_clock(xtalType);
  bflb_platform_init(0);
  bflb_platform_printf("MCU SDK:%s\r\n",&UNK_22017218);
  bflb_platform_printf("BSP Driver:%s\r\n",&UNK_22017254);
  bflb_platform_printf("BSP Common:%s\r\n",&UNK_22017290);
  bflb_platform_printf("Xtal=%d\r\n",(uint)xtalType);
  uVar1 = uVar1 & 3;
  bflb_platform_printf("flash pin=%d\r\n",uVar1);
  bflb_eflash_loader_interface_init();
  bflb_set_low_speed_flash_clock();
  SFlash_Init((SF_Ctrl_Cfg_Type *)&swCfg);
  bflb_spi_flash_init(0,uVar1,'\x01');
  SFlash_GetJedecId(&flashCfg_Gd_Q80E_Q16E,(uint8_t *)&jid);
  bflb_platform_printf("Flash ID=%x\r\n",jid);
  if (xtalType < 7) {
    bflb_platform_printf("F80M\n");
    sfCtrlCfg._0_4_ = CONCAT22(sfCtrlCfg._2_2_,0x101);
    swCfg = swCfg & 0xffffff;
    SFlash_Init((SF_Ctrl_Cfg_Type *)&swCfg);
    bflb_set_high_speed_flash_clock();
  }
  else {
    bflb_platform_printf("F16M\n");
    SFlash_Init((SF_Ctrl_Cfg_Type *)&swCfg);
  }
  bflb_platform_printf("QE\n");
  BVar2 = SFlash_Qspi_Enable(&flashCfg_Gd_Q80E_Q16E);
  if (CONCAT31(extraout_var,BVar2) != 0) {
    bflb_platform_printf("fail\n");
  }
  bflb_eflash_loader_main();
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t bflb_eflash_loader_cmd_get_bootinfo(uint16_t cmd,uint8_t *data,uint16_t len)

{
  undefined2 in_register_00002032;
  
  bflb_platform_printf("get bootinfo\r\n",CONCAT22(in_register_00002032,len));
  eflash_loader_readbuf[1][0] = 0x144b4f;
  eflash_loader_readbuf[1][1] = 0xffffffff;
  memcpy(eflash_loader_readbuf[1] + 2,&otp_cfg,0x10);
  (*eflash_loader_if->boot_if_send)((uint32_t *)0x42024134,0x18);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t bflb_eflash_loader_cmd_read_jedec_id(uint16_t cmd,uint8_t *data,uint16_t len)

{
  bflb_platform_printf("JID\n");
  eflash_loader_readbuf[1][0] = 0x44b4f;
  eflash_loader_readbuf[1][1] = 0;
  bflb_spi_flash_read_jedec_id((uint8_t *)(eflash_loader_readbuf[1] + 1));
  (*eflash_loader_if->boot_if_send)((uint32_t *)0x42024134,8);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t bflb_eflash_loader_cmd_read_log(uint16_t cmd,uint8_t *data,uint16_t len)

{
  uint32_t uVar1;
  
  bflb_platform_printf("RL\n");
  eflash_loader_readbuf[1][0] = 0x4b4f;
  uVar1 = bflb_platform_get_log((uint8_t *)(eflash_loader_readbuf[1] + 1),0x4004);
  eflash_loader_readbuf[1][0] = eflash_loader_readbuf[1][0] & 0xffff | uVar1 << 0x10;
  (*eflash_loader_if->boot_if_send)((uint32_t *)0x42024134,uVar1 + 4);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t bflb_eflash_loader_cmd_xip_read_flash_start(uint16_t cmd,uint8_t *data,uint16_t len)

{
  bflb_platform_printf("xip\n");
  bflb_spi_flash_xip_read_init();
  eflash_loader_cmd_ack_buf[0] = 0x4b4f;
  (*eflash_loader_if->boot_if_send)(eflash_loader_cmd_ack_buf,2);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t bflb_eflash_loader_cmd_xip_read_flash_finish(uint16_t cmd,uint8_t *data,uint16_t len)

{
  bflb_platform_printf("exit\n");
  bflb_spi_flash_xip_read_exit();
  eflash_loader_cmd_ack_buf[0] = 0x4b4f;
  (*eflash_loader_if->boot_if_send)(eflash_loader_cmd_ack_buf,2);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t bflb_eflash_loader_cmd_reset(uint16_t cmd,uint8_t *data,uint16_t len)

{
  bflb_platform_printf("RST\n");
  eflash_loader_cmd_ack_buf[0] = 0x4b4f;
  (*eflash_loader_if->boot_if_send)(eflash_loader_cmd_ack_buf,2);
  (*eflash_loader_if->boot_if_wait_tx_idle)(4);
  bflb_platform_delay_ms(1);
  GLB_SW_System_Reset();
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



void bflb_eflash_loader_cmd_ack(uint32_t result)

{
  if (result != 0) {
    eflash_loader_cmd_ack_buf[0] = result << 0x10 | 0x4c46;
                    // WARNING: Could not recover jumptable at 0x22010d56. Too many branches
                    // WARNING: Treating indirect jump as call
    (*eflash_loader_if->boot_if_send)(eflash_loader_cmd_ack_buf,4);
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return;
  }
  eflash_loader_cmd_ack_buf[0] = 0x4b4f;
                    // WARNING: Could not recover jumptable at 0x22010d68. Too many branches
                    // WARNING: Treating indirect jump as call
  (*eflash_loader_if->boot_if_send)(eflash_loader_cmd_ack_buf,2);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



int32_t bflb_eflash_loader_cmd_write_mem(uint16_t cmd,uint8_t *data,uint16_t len)

{
  undefined2 in_register_00002032;
  undefined4 *puVar1;
  
  bflb_platform_printf("write\r\n");
  if ((len & 7) == 0) {
    if (CONCAT22(in_register_00002032,len) != 0) {
      puVar1 = (undefined4 *)(data + (CONCAT22(in_register_00002032,len) - 1U & 0xfffffff8) + 8);
      do {
        **(undefined4 **)data = *(undefined4 *)((int)data + 4);
        data = (uint8_t *)((int)data + 8);
      } while (puVar1 != (undefined4 *)data);
    }
    bflb_eflash_loader_cmd_ack(0);
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return 0;
  }
  bflb_eflash_loader_cmd_ack(0x401);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0x401;
}



int32_t bflb_eflash_loader_cmd_write_flash_check(uint16_t cmd,uint8_t *data,uint16_t len)

{
  bflb_platform_printf("WC\n");
  bflb_eflash_loader_cmd_ack(eflash_loader_error);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

int32_t bflb_eflash_loader_cmd_read_efuse(uint16_t cmd,uint8_t *data,uint16_t len)

{
  uint len_00;
  uint index;
  undefined2 in_register_00002032;
  uint uVar1;
  int32_t iVar2;
  uint32_t result;
  uint32_t startaddr;
  uint32_t read_len;
  
  bflb_platform_printf("R\n");
  BL602_Delay_US(100);
  HBN_Set_ROOT_CLK_Sel(xtalType != GLB_PLL_XTAL_RC32M);
  result = 0x404;
  _DAT_4000f108 = xtal_clk;
  iVar2 = 0x404;
  if (CONCAT22(in_register_00002032,len) == 8) {
    index = *(uint *)data & 0xfffffffc;
    uVar1 = (uint)data[7] << 0x18 | *(uint3 *)(data + 4) & 0xfffffffc;
    eflash_loader_readbuf[1][0] = 0x4b4f;
    result = 0x405;
    iVar2 = 0x405;
    if (index < 0x80) {
      len_00 = 0x80 - index;
      if (uVar1 < 0x80 - index) {
        len_00 = uVar1;
      }
      eflash_loader_readbuf[1][0] = (uint32_t)CONCAT12((char)len_00,0x4b4f);
      bflb_efuse_read(index,eflash_loader_readbuf[1] + 1,len_00);
      iVar2 = 0;
      bflb_set_high_speed_system_clock(xtalType);
      BL602_Delay_US(100);
      (*eflash_loader_if->boot_if_send)((uint32_t *)0x42024134,len_00 + 4);
      goto LAB_22010e8c;
    }
  }
  bflb_set_high_speed_system_clock(xtalType);
  BL602_Delay_US(100);
  bflb_eflash_loader_cmd_ack(result);
LAB_22010e8c:
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return iVar2;
}



// WARNING: Variable defined which should be unmapped: startaddr
// WARNING: Globals starting with '_' overlap smaller symbols at the same address

int32_t bflb_eflash_loader_cmd_write_efuse(uint16_t cmd,uint8_t *data,uint16_t len)

{
  int32_t iVar1;
  uint len_00;
  uint32_t result;
  undefined2 in_register_00002032;
  uint uStack20;
  uint32_t startaddr;
  
  bflb_platform_printf("W\n");
  BL602_Delay_US(100);
  HBN_Set_ROOT_CLK_Sel(xtalType != GLB_PLL_XTAL_RC32M);
  _DAT_4000f108 = xtal_clk;
  if (CONCAT22(in_register_00002032,len) < 5) {
    result = 0x401;
    iVar1 = 0x401;
  }
  else {
    uStack20 = (uint)data[3] << 0x18;
    uStack20 = uStack20 | *(uint3 *)data & 0xfffffffc;
    if (uStack20 < 0x80) {
      len_00 = CONCAT22(in_register_00002032,len) - 4 & 0xfffffffc;
      if (len_00 < 0x80 - uStack20) {
        bflb_efuse_write(uStack20,(uint32_t *)data,len_00);
        result = 0;
        iVar1 = 0;
      }
      else {
        bflb_efuse_write(uStack20,(uint32_t *)data,0x80 - uStack20);
        result = 0;
        iVar1 = 0;
      }
    }
    else {
      result = 0x402;
      iVar1 = 0x402;
    }
  }
  bflb_set_high_speed_system_clock(xtalType);
  BL602_Delay_US(100);
  bflb_eflash_loader_cmd_ack(result);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return iVar1;
}



int32_t bflb_eflash_loader_cmd_flash_chip_erase(uint16_t cmd,uint8_t *data,uint16_t len)

{
  int32_t iVar1;
  
  bflb_platform_printf("CE\n");
  iVar1 = bflb_spi_flash_chiperase();
  if (iVar1 == 0) {
    bflb_eflash_loader_cmd_ack(0);
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return 0;
  }
  bflb_platform_printf("fail\n");
  bflb_eflash_loader_cmd_ack(3);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 3;
}



int32_t bflb_eflash_loader_cmd_erase_flash(uint16_t cmd,uint8_t *data,uint16_t len)

{
  uint32_t endaddr_00;
  uint len_00;
  uint32_t uVar1;
  undefined2 in_register_00002032;
  char *pcVar2;
  uint uVar3;
  int32_t iVar4;
  uint uVar5;
  uint uVar6;
  uint32_t addr;
  uint32_t startaddr;
  uint32_t endaddr;
  
  bflb_platform_printf("E\n");
  uVar1 = 2;
  iVar4 = 2;
  if (CONCAT22(in_register_00002032,len) == 8) {
    uVar1 = *(uint32_t *)data;
    endaddr_00 = *(uint32_t *)(data + 4);
    eflash_loader_error = 0;
    if (((char)jid == -0x11) && (uVar5 = (endaddr_00 + 1) - uVar1, 0x27ff < uVar5)) {
      uVar6 = 0;
      addr = uVar1;
      do {
        len_00 = uVar5 - uVar6;
        if (0x4008 < uVar5 - uVar6) {
          len_00 = 0x4008;
        }
        bflb_spi_flash_read_sahb(addr,(uint32_t *)eflash_loader_readbuf[1],len_00);
        if (uVar5 != uVar6) {
          uVar3 = 0;
          do {
            pcVar2 = (char *)((int)eflash_loader_readbuf[1] + uVar3);
            uVar3 = uVar3 + 1;
            if (*pcVar2 != -1) goto LAB_22011172;
          } while (uVar3 < len_00);
        }
        uVar6 = uVar6 + len_00;
        addr = addr + len_00;
      } while (uVar6 < uVar5);
    }
    else {
LAB_22011172:
      iVar4 = bflb_spi_flash_erase(uVar1,endaddr_00);
      if (iVar4 != 0) {
        bflb_platform_printf("fail\n");
        iVar4 = 3;
        uVar1 = 3;
        goto LAB_2201109c;
      }
    }
    uVar1 = 0;
    iVar4 = 0;
  }
LAB_2201109c:
  bflb_eflash_loader_cmd_ack(uVar1);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return iVar4;
}



int32_t bflb_eflash_loader_cmd_flash_boot(uint16_t cmd,uint8_t *data,uint16_t len)

{
  bflb_platform_printf("bf\n");
  if ((eflash_loader_if->if_type & 0xfd) == 0) {
    eflash_loader_cmd_ack_buf[0] = 0x4b4f;
    (*eflash_loader_if->boot_if_send)(eflash_loader_cmd_ack_buf,2);
  }
  bflb_eflash_loader_boot_main();
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t bflb_eflash_loader_cmd_read_mem(uint16_t cmd,uint8_t *data,uint16_t len)

{
  byte bVar1;
  uint uVar2;
  undefined2 in_register_00002032;
  undefined4 *puVar3;
  uint32_t *puVar4;
  uint32_t *puVar5;
  uint uStack24;
  uint32_t startaddr;
  uint32_t read_len;
  
  bflb_platform_printf("RM\n");
  if (CONCAT22(in_register_00002032,len) != 8) {
    eflash_loader_cmd_ack_buf[0] = 0x4044c46;
    (*eflash_loader_if->boot_if_send)(eflash_loader_cmd_ack_buf,4);
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return 0x404;
  }
  bVar1 = data[4];
  uStack24 = (uint)*(ushort *)(data + 2) << 0x10;
  startaddr = (uint)*(uint3 *)(data + 5) << 8;
  uVar2 = startaddr | bVar1 & 0xfffffffc;
  puVar3 = (undefined4 *)(uStack24 | *(ushort *)data & 0xfffffffc);
  eflash_loader_readbuf[1][0] = CONCAT22((short)uVar2,0x4b4f);
  if (uVar2 != 0) {
    puVar4 = eflash_loader_readbuf[1] + 1;
    do {
      puVar5 = puVar4 + 1;
      *(char *)puVar4 = (char)*puVar3;
      *(char *)((int)puVar4 + 1) = (char)((uint)*puVar3 >> 8);
      *(char *)((int)puVar4 + 2) = (char)*(undefined2 *)((int)puVar3 + 2);
      *(undefined *)((int)puVar4 + 3) = *(undefined *)((int)puVar3 + 3);
      puVar4 = puVar5;
    } while ((uint32_t *)((int)eflash_loader_readbuf[1] + (startaddr | bVar1 & 0xfffffffc) + 4) !=
             puVar5);
  }
  (*eflash_loader_if->boot_if_send)((uint32_t *)0x42024134,uVar2 + 4);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t bflb_eflash_loader_cmd_write_flash(uint16_t cmd,uint8_t *data,uint16_t len)

{
  int32_t iVar1;
  uint32_t uVar2;
  undefined2 in_register_00002032;
  uint32_t startaddr;
  
  bflb_platform_printf("W\n");
  if (CONCAT22(in_register_00002032,len) < 5) {
    uVar2 = 4;
    iVar1 = 4;
  }
  else {
    uVar2 = *(uint32_t *)data;
    bflb_platform_clear_time();
    if (uVar2 == 0xffffffff) {
      uVar2 = 5;
      iVar1 = 5;
    }
    else {
      if (eflash_loader_if->if_type == '\0') {
        eflash_loader_cmd_ack_buf[0] = 0x4b4f;
        (*eflash_loader_if->boot_if_send)(eflash_loader_cmd_ack_buf,2);
      }
      iVar1 = bflb_spi_flash_program(uVar2,data + 4,CONCAT22(in_register_00002032,len) - 4);
      if (iVar1 == 0) {
        uVar2 = 0;
        if (eflash_loader_if->if_type == '\0') goto LAB_22011364;
      }
      else {
        bflb_platform_printf("fail\n");
        eflash_loader_error = 6;
        uVar2 = 6;
        iVar1 = 6;
      }
    }
  }
  bflb_eflash_loader_cmd_ack(uVar2);
LAB_22011364:
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return iVar1;
}



// WARNING: Could not reconcile some variable overlaps

int32_t bflb_eflash_loader_cmd_write_status_register(uint16_t cmd,uint8_t *data,uint16_t len)

{
  BL_Err_Type BVar1;
  undefined3 extraout_var;
  undefined2 in_register_00002032;
  uint32_t writeRegCmd;
  uint32_t write_len;
  uint32_t writeData;
  
  bflb_platform_printf("WSR\n");
  if (CONCAT22(in_register_00002032,len) == 0xc) {
    write_len._0_1_ = data[8];
    write_len._1_1_ = data[9];
    write_len._2_1_ = data[10];
    write_len._3_1_ = data[0xb];
    eflash_loader_readbuf[1][0] = (uint32_t)CONCAT12(data[4],0x4b4f);
    BVar1 = bflb_spi_flash_write_status_reg_with_cmd(*data,(uint8_t *)&write_len,data[4]);
    if (CONCAT31(extraout_var,BVar1) != 1) {
      eflash_loader_cmd_ack_buf[0] = 0x4b4f;
      (*eflash_loader_if->boot_if_send)(eflash_loader_cmd_ack_buf,2);
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return 0;
    }
  }
  eflash_loader_cmd_ack_buf[0] = 0xa4c46;
  (*eflash_loader_if->boot_if_send)(eflash_loader_cmd_ack_buf,4);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 10;
}



int32_t bflb_eflash_loader_cmd_read_status_register(uint16_t cmd,uint8_t *data,uint16_t len)

{
  uint8_t regLen;
  int iVar1;
  BL_Err_Type BVar2;
  undefined3 extraout_var;
  undefined2 in_register_00002032;
  uint32_t readRegCmd;
  uint32_t read_len;
  
  bflb_platform_printf("RSR\n");
  if (CONCAT22(in_register_00002032,len) != 8) {
    eflash_loader_cmd_ack_buf[0] = 0x94c46;
    (*eflash_loader_if->boot_if_send)(eflash_loader_cmd_ack_buf,4);
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return 9;
  }
  iVar1 = *(int *)(data + 4);
  regLen = (uint8_t)*(undefined3 *)(data + 4);
  eflash_loader_readbuf[1][0] = (uint32_t)CONCAT12(regLen,0x4b4f);
  BVar2 = bflb_spi_flash_read_status_reg_with_cmd
                    (*data,(uint8_t *)(eflash_loader_readbuf[1] + 1),regLen);
  if (CONCAT31(extraout_var,BVar2) != 1) {
    (*eflash_loader_if->boot_if_send)((uint32_t *)0x42024134,iVar1 + 4);
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



// WARNING: Variable defined which should be unmapped: read_len
// WARNING: Globals starting with '_' overlap smaller symbols at the same address

int32_t bflb_eflash_loader_cmd_read_flash(uint16_t cmd,uint8_t *data,uint16_t len)

{
  undefined3 uVar1;
  uint uVar2;
  uint32_t len_00;
  uint32_t uVar3;
  uint32_t *puVar4;
  undefined2 in_register_00002032;
  undefined uVar5;
  uint len_01;
  undefined uVar6;
  uint32_t startaddr;
  uint32_t read_len;
  
  bflb_platform_printf("R\n");
  if (eflash_loader_if->if_type == '\x01') {
    if (CONCAT22(in_register_00002032,len) == 8) {
      _DAT_4201c002 = *(undefined2 *)(data + 4);
      len_00 = *(uint32_t *)(data + 4);
      DAT_4201c000 = 0x4b4f;
      uVar3 = len_00 + 4;
      bflb_spi_flash_read_sahb(*(uint32_t *)data,(uint32_t *)&DAT_4201c004,len_00);
      puVar4 = (uint32_t *)&DAT_4201c000;
      goto LAB_22011664;
    }
  }
  else if (CONCAT22(in_register_00002032,len) == 8) {
    uVar1 = *(undefined3 *)(data + 4);
    uVar2 = *(uint *)(data + 4);
    if ((eflash_loader_if->if_type == '\0') && (len_01 = 0x4004, 0x4004 < uVar2)) {
      uVar3 = 0x4008;
      uVar5 = 0x40;
      uVar6 = 4;
    }
    else {
      uVar6 = (undefined)uVar1;
      uVar5 = (undefined)((uint3)uVar1 >> 8);
      uVar3 = uVar2 + 4;
      len_01 = uVar2;
    }
    eflash_loader_readbuf[1][0] = CONCAT13(uVar5,CONCAT12(uVar6,0x4b4f));
    bflb_spi_flash_read_sahb(*(uint32_t *)data,eflash_loader_readbuf[1] + 1,len_01);
    puVar4 = (uint32_t *)0x42024134;
LAB_22011664:
    (*eflash_loader_if->boot_if_send)(puVar4,uVar3);
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return 0;
  }
  eflash_loader_cmd_ack_buf[0] = 0x44c46;
  (*eflash_loader_if->boot_if_send)(eflash_loader_cmd_ack_buf,4);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 4;
}



// WARNING: Variable defined which should be unmapped: read_len
// WARNING: Globals starting with '_' overlap smaller symbols at the same address

int32_t bflb_eflash_loader_cmd_xip_read_flash(uint16_t cmd,uint8_t *data,uint16_t len)

{
  undefined3 uVar1;
  uint uVar2;
  uint32_t len_00;
  uint32_t uVar3;
  uint32_t *puVar4;
  undefined2 in_register_00002032;
  undefined uVar5;
  uint len_01;
  undefined uVar6;
  uint32_t startaddr;
  uint32_t read_len;
  
  bflb_platform_printf("XR\n");
  if (eflash_loader_if->if_type == '\x01') {
    if (CONCAT22(in_register_00002032,len) == 8) {
      _DAT_4201c002 = *(undefined2 *)(data + 4);
      len_00 = *(uint32_t *)(data + 4);
      DAT_4201c000 = 0x4b4f;
      uVar3 = len_00 + 4;
      bflb_spi_flash_read_xip(*(uint32_t *)data,(uint32_t *)&DAT_4201c004,len_00);
      puVar4 = (uint32_t *)&DAT_4201c000;
      goto LAB_220117dc;
    }
  }
  else if (CONCAT22(in_register_00002032,len) == 8) {
    uVar1 = *(undefined3 *)(data + 4);
    uVar2 = *(uint *)(data + 4);
    if ((eflash_loader_if->if_type == '\0') && (len_01 = 0x4004, 0x4004 < uVar2)) {
      uVar3 = 0x4008;
      uVar5 = 0x40;
      uVar6 = 4;
    }
    else {
      uVar6 = (undefined)uVar1;
      uVar5 = (undefined)((uint3)uVar1 >> 8);
      uVar3 = uVar2 + 4;
      len_01 = uVar2;
    }
    eflash_loader_readbuf[1][0] = CONCAT13(uVar5,CONCAT12(uVar6,0x4b4f));
    bflb_spi_flash_read_xip(*(uint32_t *)data,eflash_loader_readbuf[1] + 1,len_01);
    puVar4 = (uint32_t *)0x42024134;
LAB_220117dc:
    (*eflash_loader_if->boot_if_send)(puVar4,uVar3);
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return 0;
  }
  eflash_loader_cmd_ack_buf[0] = 0x44c46;
  (*eflash_loader_if->boot_if_send)(eflash_loader_cmd_ack_buf,4);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 4;
}



int32_t bflb_eflash_loader_cmd_readSha_flash(uint16_t cmd,uint8_t *data,uint16_t len)

{
  uint32_t addr;
  short sVar1;
  uint len_00;
  int32_t iVar2;
  undefined2 in_register_00002032;
  uint32_t (*pauVar3) [4098];
  uint32_t startaddr;
  uint32_t read_len;
  SEC_Eng_SHA256_Ctx shaCtx;
  
  bflb_platform_printf("RSha\n");
  pauVar3 = (uint32_t (*) [4098])&DAT_4201c000;
  if (eflash_loader_if->if_type != '\x01') {
    pauVar3 = eflash_loader_readbuf[1];
  }
  if (CONCAT22(in_register_00002032,len) == 8) {
    addr = *(uint32_t *)data;
    len_00 = *(uint *)(data + 4);
    Sec_Eng_SHA256_Init((SEC_Eng_SHA256_Ctx *)&read_len,SEC_ENG_SHA_ID0,SEC_ENG_SHA256,shaTmpBuf,
                        padding);
    Sec_Eng_SHA_Start(SEC_ENG_SHA_ID0);
    if (len_00 != 0) {
      for (; 0x4008 < len_00; len_00 = len_00 - 0x4008) {
        bflb_spi_flash_read_sahb(addr,ShaInbuf,0x4008);
        Sec_Eng_SHA256_Update
                  ((SEC_Eng_SHA256_Ctx *)&read_len,SEC_ENG_SHA_ID0,(uint8_t *)ShaInbuf,0x4008);
        addr = addr + 0x4008;
      }
      bflb_spi_flash_read_sahb(addr,ShaInbuf,len_00);
      Sec_Eng_SHA256_Update
                ((SEC_Eng_SHA256_Ctx *)&read_len,SEC_ENG_SHA_ID0,(uint8_t *)ShaInbuf,len_00);
    }
    Sec_Eng_SHA256_Finish((SEC_Eng_SHA256_Ctx *)&read_len,SEC_ENG_SHA_ID0,(uint8_t *)(*pauVar3 + 1))
    ;
    sVar1 = 0x20;
    do {
      sVar1 = sVar1 + -1;
      bflb_platform_printf("\r\n");
    } while (sVar1 != 0);
    (*pauVar3)[0] = 0x204b4f;
    (*eflash_loader_if->boot_if_send)((uint32_t *)pauVar3,0x24);
    iVar2 = 0;
  }
  else {
    eflash_loader_cmd_ack_buf[0] = 0x44c46;
    (*eflash_loader_if->boot_if_send)(eflash_loader_cmd_ack_buf,4);
    iVar2 = 4;
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return iVar2;
}



int32_t bflb_eflash_loader_cmd_xip_readSha_flash(uint16_t cmd,uint8_t *data,uint16_t len)

{
  uint32_t addr;
  short sVar1;
  uint len_00;
  int32_t iVar2;
  undefined2 in_register_00002032;
  uint32_t (*pauVar3) [4098];
  uint32_t startaddr;
  uint32_t read_len;
  SEC_Eng_SHA256_Ctx shaCtx;
  
  bflb_platform_printf("XRSha\n");
  pauVar3 = (uint32_t (*) [4098])&DAT_4201c000;
  if (eflash_loader_if->if_type != '\x01') {
    pauVar3 = eflash_loader_readbuf[1];
  }
  if (CONCAT22(in_register_00002032,len) == 8) {
    addr = *(uint32_t *)data;
    len_00 = *(uint *)(data + 4);
    Sec_Eng_SHA256_Init((SEC_Eng_SHA256_Ctx *)&read_len,SEC_ENG_SHA_ID0,SEC_ENG_SHA256,shaTmpBuf,
                        padding);
    Sec_Eng_SHA_Start(SEC_ENG_SHA_ID0);
    if (len_00 != 0) {
      for (; 0x4008 < len_00; len_00 = len_00 - 0x4008) {
        bflb_spi_flash_read_xip(addr,ShaInbuf,0x4008);
        Sec_Eng_SHA256_Update
                  ((SEC_Eng_SHA256_Ctx *)&read_len,SEC_ENG_SHA_ID0,(uint8_t *)ShaInbuf,0x4008);
        addr = addr + 0x4008;
      }
      bflb_spi_flash_read_xip(addr,ShaInbuf,len_00);
      Sec_Eng_SHA256_Update
                ((SEC_Eng_SHA256_Ctx *)&read_len,SEC_ENG_SHA_ID0,(uint8_t *)ShaInbuf,len_00);
    }
    Sec_Eng_SHA256_Finish((SEC_Eng_SHA256_Ctx *)&read_len,SEC_ENG_SHA_ID0,(uint8_t *)(*pauVar3 + 1))
    ;
    sVar1 = 0x20;
    do {
      sVar1 = sVar1 + -1;
      bflb_platform_printf("\r\n");
    } while (sVar1 != 0);
    (*pauVar3)[0] = 0x204b4f;
    (*eflash_loader_if->boot_if_send)((uint32_t *)pauVar3,0x24);
    iVar2 = 0;
  }
  else {
    eflash_loader_cmd_ack_buf[0] = 0x44c46;
    (*eflash_loader_if->boot_if_send)(eflash_loader_cmd_ack_buf,4);
    iVar2 = 4;
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return iVar2;
}



int32_t bflb_eflash_loader_cmd_read_mac_addr(uint16_t cmd,uint8_t *data,uint16_t len)

{
  BL_Err_Type BVar1;
  undefined3 extraout_var;
  uint32_t uVar2;
  undefined2 in_register_00002032;
  uint32_t crc;
  
  bflb_platform_printf("RMA\n",CONCAT22(in_register_00002032,len));
  eflash_loader_readbuf[1][0] = 0xa4b4f;
  BVar1 = EF_Ctrl_Read_MAC_Address((uint8_t *)(eflash_loader_readbuf[1] + 1));
  if (CONCAT31(extraout_var,BVar1) == 0) {
    uVar2 = BFLB_Soft_CRC32(eflash_loader_readbuf[1] + 1,6);
    eflash_loader_readbuf[1][2]._2_2_ = (undefined2)uVar2;
    eflash_loader_readbuf[1][3]._0_2_ = (undefined2)(uVar2 >> 0x10);
    (*eflash_loader_if->boot_if_send)((uint32_t *)0x42024134,0xe);
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return 0;
  }
  eflash_loader_cmd_ack_buf[0] = 0x4074c46;
  (*eflash_loader_if->boot_if_send)(eflash_loader_cmd_ack_buf,4);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0x407;
}



// WARNING: Could not reconcile some variable overlaps

int32_t bflb_eflash_loader_cmd_set_flash_para(uint16_t cmd,uint8_t *data,uint16_t len)

{
  byte bVar1;
  uint8_t div;
  undefined2 in_register_00002032;
  uint8_t ioMode;
  undefined4 uStack44;
  SF_Ctrl_Cfg_Type sfCtrlCfg;
  
  bflb_platform_printf("SP\n");
  sfCtrlCfg._0_4_ = 0x101;
  uStack44 = 0;
  sfCtrlCfg.clkInvert = '\0';
  if (CONCAT22(in_register_00002032,len) == 4) {
    bVar1 = *data;
    div = data[1];
    ioMode = data[2];
    bflb_set_low_speed_flash_clock();
    SFlash_Init((SF_Ctrl_Cfg_Type *)&uStack44);
    if (bVar1 != 0xff) {
      bflb_spi_flash_init(0,(uint)bVar1,'\x01');
      BL602_Delay_MS(2);
    }
  }
  else {
    if (CONCAT22(in_register_00002032,len) != 0x58) {
      eflash_loader_cmd_ack_buf[0] = 0x84c46;
      (*eflash_loader_if->boot_if_send)(eflash_loader_cmd_ack_buf,4);
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return 8;
    }
    bVar1 = *data;
    div = data[1];
    ioMode = data[2];
    bflb_set_low_speed_flash_clock();
    SFlash_Init((SF_Ctrl_Cfg_Type *)&uStack44);
    if (bVar1 != 0xff) {
      bflb_spi_flash_init(0,(uint)bVar1,'\x01');
      BL602_Delay_MS(2);
    }
    bflb_spi_flash_update_para(data + 4);
  }
  bflb_spi_flash_set_io_mode(ioMode);
  bflb_platform_printf("F80M");
  if (div == '\0') {
    uStack44 = CONCAT13(1,(undefined3)uStack44);
  }
  SFlash_Init((SF_Ctrl_Cfg_Type *)&uStack44);
  bflb_set_flash_clock_div(div);
  eflash_loader_cmd_ack_buf[0] = 0x4b4f;
  (*eflash_loader_if->boot_if_send)(eflash_loader_cmd_ack_buf,2);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t bflb_eflash_loader_cmd_write_flash_with_decompress(uint16_t cmd,uint8_t *data,uint16_t len)

{
  bool bVar1;
  size_t sVar2;
  xz_ret xVar3;
  uint8_t *data_00;
  undefined3 extraout_var;
  int32_t iVar5;
  uint8_t *buf;
  undefined2 in_register_00002032;
  uint uVar6;
  uint32_t first_decompress;
  int iVar4;
  
  data_00 = xz_BSP_get_work_buffer();
  bflb_platform_printf("CW\n");
  if (CONCAT22(in_register_00002032,len) < 5) {
    eflash_loader_cmd_ack_buf[0] = 0xb4c46;
    (*eflash_loader_if->boot_if_send)(eflash_loader_cmd_ack_buf,4);
    uVar6 = 0xb;
    goto LAB_22011e1a;
  }
  if ((int)*(uint *)data < 0) {
    bflb_eflash_loader_cmd_write_flash_with_decompress::startaddr = *(uint *)data & 0x7fffffff;
    xz_BSP_init();
    xz_crc32_init();
    buf = xz_BSP_get_dict_buffer();
    simple_malloc_init(buf,0x10000);
    bflb_eflash_loader_cmd_write_flash_with_decompress::s = xz_dec_init(XZ_PREALLOC,0x8000);
    if (bflb_eflash_loader_cmd_write_flash_with_decompress::s != (xz_dec *)0x0) {
      bflb_eflash_loader_cmd_write_flash_with_decompress::b.out_pos = 0;
      bflb_eflash_loader_cmd_write_flash_with_decompress::b.out = data_00;
      bflb_eflash_loader_cmd_write_flash_with_decompress::b.out_size = xz_BSP_get_output_size();
      goto LAB_22011d7e;
    }
    bflb_platform_printf("xz fail\n");
  }
  else {
LAB_22011d7e:
    bVar1 = (eflash_loader_if->if_type & 0xfd) == 0;
    if (bVar1) {
      eflash_loader_cmd_ack_buf[0] = 0x4b4f;
      (*eflash_loader_if->boot_if_send)(eflash_loader_cmd_ack_buf,2);
    }
    uVar6 = (uint)bVar1;
    bflb_eflash_loader_cmd_write_flash_with_decompress::b.in = data + 4;
    bflb_eflash_loader_cmd_write_flash_with_decompress::b.in_pos = 0;
    bflb_eflash_loader_cmd_write_flash_with_decompress::b.in_size =
         CONCAT22(in_register_00002032,len) - 4;
    do {
      while( true ) {
        if (bflb_eflash_loader_cmd_write_flash_with_decompress::b.in_pos ==
            bflb_eflash_loader_cmd_write_flash_with_decompress::b.in_size) goto LAB_22011f12;
        xVar3 = xz_dec_run(bflb_eflash_loader_cmd_write_flash_with_decompress::s,
                           &bflb_eflash_loader_cmd_write_flash_with_decompress::b);
        iVar4 = CONCAT31(extraout_var,xVar3);
        if (bflb_eflash_loader_cmd_write_flash_with_decompress::b.out_pos ==
            bflb_eflash_loader_cmd_write_flash_with_decompress::b.out_size) break;
        if (iVar4 != 0) goto LAB_22011de0;
      }
      iVar5 = bflb_spi_flash_program
                        (bflb_eflash_loader_cmd_write_flash_with_decompress::startaddr,data_00,
                         bflb_eflash_loader_cmd_write_flash_with_decompress::b.out_pos);
      sVar2 = bflb_eflash_loader_cmd_write_flash_with_decompress::b.out_pos;
      if (iVar5 != 0) goto LAB_22011e46;
      bflb_eflash_loader_cmd_write_flash_with_decompress::b.out_pos = 0;
      bflb_eflash_loader_cmd_write_flash_with_decompress::startaddr =
           sVar2 + bflb_eflash_loader_cmd_write_flash_with_decompress::startaddr;
    } while (iVar4 == 0);
LAB_22011de0:
    if ((bflb_eflash_loader_cmd_write_flash_with_decompress::b.out_pos != 0) &&
       (iVar5 = bflb_spi_flash_program
                          (bflb_eflash_loader_cmd_write_flash_with_decompress::startaddr,data_00,
                           bflb_eflash_loader_cmd_write_flash_with_decompress::b.out_pos),
       iVar5 != 0)) {
LAB_22011e46:
      bflb_platform_printf("fail\n");
      eflash_loader_cmd_ack_buf[0] = 0xb4c46;
      eflash_loader_error = 0xb;
      uVar6 = 0xb;
      (*eflash_loader_if->boot_if_send)(eflash_loader_cmd_ack_buf,4);
      goto LAB_22011e1a;
    }
    if (iVar4 == 1) {
      xz_dec_end(bflb_eflash_loader_cmd_write_flash_with_decompress::s);
LAB_22011f12:
      if (uVar6 == 0) {
        eflash_loader_cmd_ack_buf[0] = 0x4b4f;
        (*eflash_loader_if->boot_if_send)(eflash_loader_cmd_ack_buf,2);
      }
      else {
        uVar6 = 0;
      }
      goto LAB_22011e1a;
    }
    xz_dec_end(bflb_eflash_loader_cmd_write_flash_with_decompress::s);
    bflb_platform_printf("fail\n");
  }
  eflash_loader_cmd_ack_buf[0] = 0xc4c46;
  (*eflash_loader_if->boot_if_send)(eflash_loader_cmd_ack_buf,4);
  uVar6 = 0xc;
LAB_22011e1a:
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return uVar6;
}



void bflb_eflash_loader_cmd_init(void)

{
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  eflash_loader_error = 0;
  return;
}



// WARNING: Exceeded maximum restarts with more pending

int32_t bflb_eflash_loader_cmd_process(uint8_t cmdid,uint8_t *data,uint16_t len)

{
  undefined3 in_register_00002029;
  int32_t iVar1;
  int iVar2;
  eflash_loader_cmd_cfg_t *peVar3;
  
  peVar3 = eflash_loader_cmds;
  iVar2 = 0;
  if (CONCAT31(in_register_00002029,cmdid) != 0x10) {
    do {
      peVar3 = peVar3 + 1;
      iVar2 = iVar2 + 1;
      if (iVar2 == 0x18) {
        gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
        return 0;
      }
    } while ((uint)peVar3->cmd != CONCAT31(in_register_00002029,cmdid));
  }
  if ((eflash_loader_cmds[iVar2].enabled == '\x01') &&
     (eflash_loader_cmds[iVar2].cmd_process != (pfun_cmd_process *)0x0)) {
                    // WARNING: Could not recover jumptable at 0x22011f8e. Too many branches
                    // WARNING: Treating indirect jump as call
    iVar1 = (*eflash_loader_cmds[iVar2].cmd_process)();
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return iVar1;
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0x101;
}



// WARNING: Could not reconcile some variable overlaps

uint8_t bootrom_read_boot_mode(void)

{
  int iVar1;
  uint uVar2;
  uint32_t uVar3;
  undefined4 uStack24;
  GLB_GPIO_Cfg_Type cfg;
  
  uStack24 = 0x2000b08;
  cfg._0_2_ = 0x100;
  iVar1 = 3;
  GLB_GPIO_Init((GLB_GPIO_Cfg_Type *)&uStack24);
  uVar2 = 0;
  do {
    BL602_Delay_US(1);
    uVar3 = GLB_GPIO_Read(GLB_GPIO_PIN_8);
    iVar1 = iVar1 + -1;
    uVar2 = uVar2 + (uVar3 == 0);
  } while (iVar1 != 0);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return uVar2 < 2;
}



// WARNING: Could not reconcile some variable overlaps

void bflb_eflash_loader_init_uart_gpio(uint8_t eflash_loader_uart_pin_select)

{
  undefined4 uStack24;
  GLB_GPIO_Cfg_Type cfg;
  
  uStack24 = 0x707;
  cfg._0_2_ = 0x103;
  GLB_GPIO_Init((GLB_GPIO_Cfg_Type *)&uStack24);
  uStack24 = 0x10710;
  GLB_GPIO_Init((GLB_GPIO_Cfg_Type *)&uStack24);
  GLB_UART_Fun_Sel(GLB_UART_SIG_7,GLB_UART_SIG_FUN_UART0_RXD);
  GLB_UART_Fun_Sel(GLB_UART_SIG_0,GLB_UART_SIG_FUN_UART0_TXD);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Could not reconcile some variable overlaps

void bflb_eflash_loader_init_sdio_gpio(uint8_t eflash_loader_sdio_pin_select)

{
  uint uVar1;
  undefined3 in_register_00002029;
  undefined auStack56 [3];
  byte bStack53;
  GLB_GPIO_Cfg_Type cfg;
  uint8_t gpiopins [6];
  uint8_t gpiofuns [6];
  
  stack0xffffffca = 0x202;
  stack0xffffffd0 = 0;
  gpiopins._0_2_ = 0;
  stack0xffffffd8 = 0;
  cfg._0_2_ = 0x101;
  auStack56[1] = CONCAT31(in_register_00002029,eflash_loader_sdio_pin_select) == 0;
  if ((bool)auStack56[1]) {
    stack0xffffffd0 = 0x3020100;
    gpiopins._0_2_ = 0x504;
    stack0xffffffd8 = 0x1010101;
  }
  uVar1 = 0;
  auStack56[0] = '\0';
  do {
    if (uVar1 == 0) {
      stack0xffffffca = (ushort)bStack53 << 8;
      GLB_GPIO_Init((GLB_GPIO_Cfg_Type *)auStack56);
    }
    else {
      stack0xffffffca = CONCAT11(bStack53,2);
      GLB_GPIO_Init((GLB_GPIO_Cfg_Type *)auStack56);
      if ((uVar1 & 0xff) == 5) {
        gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
        return;
      }
    }
    auStack56[0] = (&cfg.smtCtrl)[uVar1];
    auStack56[1] = gpiopins[uVar1 + 5];
    uVar1 = uVar1 + 1;
  } while( true );
}



eflash_loader_if_cfg_t * bflb_eflash_loader_if_set(eflash_loader_if_type_t type)

{
  undefined3 in_register_00002029;
  int iVar1;
  
  iVar1 = CONCAT31(in_register_00002029,type);
  if (iVar1 == 1) {
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    eflash_loader_if_cfg._0_2_ = 0x201;
    eflash_loader_if_cfg.disabled = '\0';
    eflash_loader_if_cfg._4_4_ = 0x13884008;
    eflash_loader_if_cfg.boot_if_init = bflb_eflash_loader_jlink_init;
    eflash_loader_if_cfg.boot_if_handshake_poll = bflb_eflash_loader_jlink_handshake_poll;
    eflash_loader_if_cfg.boot_if_recv = bflb_eflash_loader_jlink_recv;
    eflash_loader_if_cfg.boot_if_send = bflb_eflash_loader_jlink_send;
    eflash_loader_if_cfg.boot_if_wait_tx_idle = bflb_eflash_loader_jlink_wait_tx_idle;
    eflash_loader_if_cfg.boot_if_deinit = bflb_eflash_loader_jlink_deinit;
    return &eflash_loader_if_cfg;
  }
  if (iVar1 != 2) {
    if (iVar1 != 0) {
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return (eflash_loader_if_cfg_t *)0x0;
    }
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    eflash_loader_if_cfg._0_2_ = 0x100;
    eflash_loader_if_cfg.disabled = '\0';
    eflash_loader_if_cfg._4_4_ = 0x7d04008;
    eflash_loader_if_cfg.boot_if_init = bflb_eflash_loader_uart_init;
    eflash_loader_if_cfg.boot_if_handshake_poll = bflb_eflash_loader_uart_handshake_poll;
    eflash_loader_if_cfg.boot_if_recv = bflb_eflash_loader_uart_recv;
    eflash_loader_if_cfg.boot_if_send = bflb_eflash_loader_uart_send;
    eflash_loader_if_cfg.boot_if_wait_tx_idle = bflb_eflash_loader_usart_wait_tx_idle;
    eflash_loader_if_cfg.boot_if_deinit = bflb_eflash_loader_uart_deinit;
    return &eflash_loader_if_cfg;
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  eflash_loader_if_cfg._0_2_ = 2;
  eflash_loader_if_cfg.disabled = '\0';
  eflash_loader_if_cfg._4_4_ = 0x13884008;
  eflash_loader_if_cfg.boot_if_init = bflb_eflash_loader_sdio_init;
  eflash_loader_if_cfg.boot_if_handshake_poll = bflb_eflash_loader_sdio_handshake_poll;
  eflash_loader_if_cfg.boot_if_recv = bflb_eflash_loader_sdio_recv;
  eflash_loader_if_cfg.boot_if_send = bflb_eflash_loader_sdio_send;
  eflash_loader_if_cfg.boot_if_wait_tx_idle = bflb_eflash_loader_sdio_wait_tx_idle;
  eflash_loader_if_cfg.boot_if_deinit = bflb_eflash_loader_sdio_deinit;
  return &eflash_loader_if_cfg;
}



int32_t bflb_eflash_loader_if_send_pending(void)

{
  uint32_t uStack20;
  uint32_t pending_data;
  
  uStack20 = 0x4450;
  (*eflash_loader_if->boot_if_send)(&uStack20,2);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t bflb_eflash_loader_interface_init(void)

{
  bflb_eflash_loader_init_sdio_gpio('\0');
  sdu_init();
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t bflb_eflash_loader_interface_deinit(void)

{
  sdu_deinit();
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t bflb_eflash_loader_main(void)

{
  int iVar1;
  int32_t iVar2;
  uint32_t *puVar3;
  char *pcVar4;
  char cVar5;
  uint uVar6;
  byte bVar7;
  eflash_loader_if_type_t type;
  uint32_t uStack68;
  uint32_t total_len;
  
  type = BFLB_EFLASH_LOADER_IF_JLINK;
  do {
    do {
      if (eflash_loader_if != (eflash_loader_if_cfg_t *)0x0) {
        type = eflash_loader_if->if_type_onfail;
      }
      do {
        eflash_loader_if = bflb_eflash_loader_if_set(type);
      } while (eflash_loader_if == (eflash_loader_if_cfg_t *)0x0);
      bVar7 = eflash_loader_if->disabled;
    } while ((bVar7 != 0) || (iVar2 = (*eflash_loader_if->boot_if_init)((void *)0x0), iVar2 != 0));
    do {
      iVar2 = (*eflash_loader_if->boot_if_handshake_poll)((void *)0x0);
    } while (iVar2 == 0xfffe);
    if (iVar2 == 0) {
      bflb_eflash_loader_cmd_init();
LAB_220122be:
      do {
        iVar1 = 2;
        do {
          uStack68 = 0;
          puVar3 = (*eflash_loader_if->boot_if_recv)
                             (&uStack68,(uint)eflash_loader_if->maxlen,
                              (uint)eflash_loader_if->timeout);
          if (uStack68 != 0) break;
          bflb_platform_printf(" return ");
          bflb_platform_printf("\n");
          if (iVar1 == 1) {
            bflb_platform_printf("B");
            goto LAB_220122ac;
          }
          iVar1 = 1;
        } while (uStack68 == 0);
        if (*(char *)((int)puVar3 + 1) == '\0') {
LAB_220122ea:
          iVar2 = bflb_eflash_loader_cmd_process
                            (*(uint8_t *)puVar3,(uint8_t *)(puVar3 + 1),*(ushort *)((int)puVar3 + 2)
                            );
          if (iVar2 != 0) {
            bflb_platform_printf(" CMD Pro Ret ");
            bVar7 = bVar7 + 1;
            bflb_platform_printf("\n");
            if (2 < bVar7) break;
          }
          goto LAB_220122be;
        }
        cVar5 = '\0';
        uVar6 = 2;
        do {
          pcVar4 = (char *)((int)puVar3 + uVar6);
          uVar6 = uVar6 + 1;
          cVar5 = cVar5 + *pcVar4;
        } while (uVar6 < *(ushort *)((int)puVar3 + 2) + 4);
        if (cVar5 == *(char *)((int)puVar3 + 1)) goto LAB_220122ea;
        bflb_platform_printf("Checksum error");
        bflb_platform_printf("\n");
        eflash_loader_cmd_ack_buf[0] = 0x1034c46;
        (*eflash_loader_if->boot_if_send)(eflash_loader_cmd_ack_buf,4);
      } while (*(char *)puVar3 != '@');
    }
LAB_220122ac:
    (*eflash_loader_if->boot_if_deinit)(eflash_loader_if->boot_if_deinit);
  } while( true );
}



int32_t bflb_eflash_loader_jlink_init(void *data)

{
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

int32_t bflb_eflash_loader_jlink_handshake_poll(void *data)

{
  if (_DAT_2201bff0 == 0x59445248) {
    bflb_platform_printf("SH\n");
    _DAT_2201bff0 = 0;
    BL602_MemCpy_Fast(&DAT_4201c000,&DAT_220174a8,4);
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    _DAT_2201bff0 = 0x4b434153;
    return 0;
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0xffff;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

uint32_t * bflb_eflash_loader_jlink_recv(uint32_t *recv_len,uint32_t maxlen,uint32_t timeout)

{
  uint32_t *puVar1;
  
  puVar1 = recv_len;
  bflb_platform_clear_time();
  do {
    if ((_DAT_2201bff0 == 0x59445248) || (bflb_platform_get_time_ms(), maxlen != 0)) break;
  } while (puVar1 < timeout);
  if (_DAT_2201bff0 == 0x59445248) {
    _DAT_2201bff0 = 0;
    if (CONCAT11(DAT_4201c003,DAT_4201c002) < 0x3ffd) {
      *recv_len = (uint)(ushort)(CONCAT11(DAT_4201c003,DAT_4201c002) + 4);
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return (uint32_t *)&DAT_4201c000;
    }
  }
  *recv_len = 0;
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return (uint32_t *)0x0;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

int32_t bflb_eflash_loader_jlink_send(uint32_t *data,uint32_t len)

{
  if (data != (uint32_t *)&DAT_4201c000) {
    BL602_MemCpy_Fast(&DAT_4201c000,data,len);
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    _DAT_2201bff0 = 0x4b434153;
    return 0;
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  _DAT_2201bff0 = 0x4b434153;
  return 0;
}



int32_t bflb_eflash_loader_jlink_wait_tx_idle(uint32_t timeout)

{
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t bflb_eflash_loader_jlink_deinit(void)

{
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



void bflb_eflash_loader_usart_if_init(uint32_t bdrate)

{
  BL_Sts_Type BVar1;
  undefined3 extraout_var;
  
  if (bdrate != 0) {
    uart_if_cfg.baudRate = bdrate;
  }
  GLB_AHB_Slave1_Reset(BL_AHB_SLAVE1_UART0);
  UART_IntMask(UART0_ID,UART_INT_ALL,MASK);
  clic_disable_interrupt(0x2d);
  do {
    BVar1 = UART_GetTxBusBusyStatus(UART0_ID);
  } while (CONCAT31(extraout_var,BVar1) == 1);
  UART_Disable(UART0_ID,UART_TXRX);
  UART_TxFreeRun(UART0_ID,ENABLE);
  UART_SetRxTimeoutValue(UART0_ID,'P');
  UART_IntMask(UART0_ID,UART_INT_RTO,UNMASK);
  UART_IntMask(UART0_ID,UART_INT_RX_FIFO_REQ,UNMASK);
  UART_Int_Callback_Install(UART0_ID,UART_INT_RTO,uart_if_rto_cbf);
  UART_Int_Callback_Install(UART0_ID,UART_INT_RX_FIFO_REQ,uart_if_rda_cbf);
  clic_enable_interrupt(0x2d);
  GLB_Set_UART_CLK('\x01',HBN_UART_CLK_160M,'\x03');
  UART_Init(UART0_ID,&uart_if_cfg);
  UART_FifoConfig(UART0_ID,&uart_if_fifocfg);
  UART_Enable(UART0_ID,UART_TXRX);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



void uart_if_rto_cbf(void)

{
  uint32_t uVar1;
  
  uVar1 = UART_ReceiveData(UART0_ID,(uint8_t *)
                                    ((int)eflash_loader_readbuf[rx_buf_index] + rx_buf_len),
                           0x4008 - rx_buf_len);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  rx_buf_len = rx_buf_len + uVar1;
  return;
}



uint32_t * bflb_eflash_loader_usart_if_receive(uint32_t *recv_len,uint16_t timeout,uint16_t maxlen)

{
  uint32_t uVar1;
  uint32_t (*pauVar2) [4098];
  uint32_t *puVar3;
  undefined2 in_register_0000202e;
  uint32_t *puVar4;
  uint uVar5;
  uint uVar6;
  
  uVar1 = rx_buf_index;
  pauVar2 = eflash_loader_readbuf[rx_buf_index];
  puVar3 = recv_len;
  puVar4 = (uint32_t *)CONCAT22(in_register_0000202e,timeout);
  bflb_platform_clear_time();
  do {
    if (3 < rx_buf_len) {
      uVar6 = (uint)*(ushort *)((int)eflash_loader_readbuf[uVar1] + 2);
      uVar5 = uVar6 + 4;
      if (uVar5 <= rx_buf_len) {
        rx_buf_index = rx_buf_index + 1 & 1;
        rx_buf_len = 0;
        if (uVar6 < 0x4005) {
          *recv_len = uVar5;
          gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
          return (uint32_t *)pauVar2;
        }
LAB_22012652:
        *recv_len = 0;
        gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
        return (uint32_t *)0x0;
      }
    }
    bflb_platform_get_time_ms();
    if ((puVar4 != (uint32_t *)0x0) ||
       ((uint32_t *)CONCAT22(in_register_0000202e,timeout) <= puVar3)) goto LAB_22012652;
  } while( true );
}



void uart_if_rda_cbf(void)

{
  uint32_t uVar1;
  
  uVar1 = UART_ReceiveData(UART0_ID,(uint8_t *)
                                    ((int)eflash_loader_readbuf[rx_buf_index] + rx_buf_len),
                           0x4008 - rx_buf_len);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  rx_buf_len = uVar1 + rx_buf_len;
  return;
}



int32_t bflb_eflash_loader_uart_init(void *data)

{
  BL_Sts_Type BVar1;
  undefined3 extraout_var;
  
  abr_gpio_sel = '\0';
  UART_Disable(UART0_ID,UART_TXRX);
  clic_disable_interrupt(0x2d);
  GLB_AHB_Slave1_Reset(BL_AHB_SLAVE1_UART0);
  UART_AutoBaudDetection(UART0_ID,DISABLE);
  bflb_eflash_loader_init_uart_gpio(abr_gpio_sel);
  do {
    BVar1 = UART_GetTxBusBusyStatus(UART0_ID);
  } while (CONCAT31(extraout_var,BVar1) == 1);
  UART_Disable(UART0_ID,UART_TXRX);
  GLB_Set_UART_CLK('\x01',HBN_UART_CLK_160M,'\x03');
  UART_Init(UART0_ID,&uart_if_cfg);
  UART_AutoBaudDetection(UART0_ID,ENABLE);
  UART_Enable(UART0_ID,UART_TXRX);
  bflb_platform_clear_time();
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t bflb_eflash_loader_uart_handshake_poll(void *data)

{
  BL_Sts_Type BVar1;
  uint16_t uVar2;
  undefined3 extraout_var;
  undefined2 extraout_var_00;
  int iVar4;
  uint32_t uStack20;
  uint32_t i;
  uint uVar3;
  
  iVar4 = 1;
  uStack20 = 0;
  BVar1 = UART_GetIntStatus(UART0_ID,UART_INT_RX_END);
  uVar3 = CONCAT31(extraout_var,BVar1);
  if (uVar3 == 1) {
    UART_IntClear(UART0_ID,UART_INT_RX_END);
    uVar2 = UART_GetAutoBaudCount(UART0_ID,UART_AUTOBAUD_0X55);
    detected_baudrate = 40000000 / (CONCAT22(extraout_var_00,uVar2) + 1);
    if (970000 < (int)detected_baudrate) {
      detected_baudrate = (detected_baudrate + 250000) - (detected_baudrate + 250000) % 500000;
      if (40000000 / detected_baudrate < 4) {
        gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
        return 0xffff;
      }
    }
    bflb_platform_delay_ms(0xc);
    UART_AutoBaudDetection(UART0_ID,DISABLE);
    bflb_eflash_loader_usart_if_init(detected_baudrate);
    bflb_platform_printf("BDR: %d\r\n",detected_baudrate);
    uVar2 = 2;
    UART_SendData(UART0_ID,"OK",2);
    bflb_platform_printf("C\r\n");
    bflb_eflash_loader_usart_if_receive(&uStack20,1,uVar2);
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    rx_buf_index = 0;
    rx_buf_len = 0;
    eflash_loader_readbuf[0][0] = 0;
    return 0;
  }
  detected_baudrate = 0;
  bflb_platform_get_time_ms();
  if ((iVar4 == 0) && (uVar3 < 2)) {
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return 0xfffe;
  }
  UART_AutoBaudDetection(UART0_ID,DISABLE);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0xffff;
}



uint32_t * bflb_eflash_loader_uart_recv(uint32_t *recv_len,uint32_t maxlen,uint32_t timeout)

{
  uint32_t *puVar1;
  
  puVar1 = bflb_eflash_loader_usart_if_receive(recv_len,(uint16_t)timeout,(uint16_t)timeout);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return puVar1;
}



int32_t bflb_eflash_loader_uart_send(uint32_t *data,uint32_t len)

{
  UART_SendData(UART0_ID,(uint8_t *)data,len);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t bflb_eflash_loader_usart_wait_tx_idle(uint32_t timeout)

{
  bflb_platform_delay_ms(timeout);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t bflb_eflash_loader_uart_deinit(void)

{
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t bflb_eflash_loader_sdio_init(void *data)

{
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t bflb_eflash_loader_sdio_handshake_poll(void *data)

{
  int32_t iVar1;
  
  iVar1 = sdu_host_check();
  if (-1 < iVar1) {
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return 0;
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0xffff;
}



uint32_t * bflb_eflash_loader_sdio_recv(uint32_t *recv_len,uint32_t maxlen,uint32_t timeout)

{
  uint32_t *puVar1;
  
  bflb_platform_clear_time();
  while( true ) {
    puVar1 = sdu_receive_data(recv_len);
    if (puVar1 != (uint32_t *)0x0) {
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return puVar1;
    }
    bflb_platform_get_time_ms();
    if (maxlen != 0) break;
    if (timeout <= puVar1) {
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return (uint32_t *)0x0;
    }
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return (uint32_t *)0x0;
}



int32_t bflb_eflash_loader_sdio_send(uint32_t *data,uint32_t len)

{
  sdu_send_data(data,len);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t bflb_eflash_loader_sdio_wait_tx_idle(uint32_t timeout)

{
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t bflb_eflash_loader_sdio_deinit(void)

{
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t bflb_eflash_loader_clk_set_from_user(boot_clk_cfg_t *cfg_new,uint8_t lock)

{
  GLB_PLL_XTAL_Type xtalType;
  uint32_t uVar1;
  uint32_t uVar2;
  int32_t iVar3;
  
  if (bflb_eflash_loader_clk_set_from_user::clk_locked == '\x01') {
    iVar3 = 0xffff;
  }
  else {
    if ((((*(char *)&cfg_new->magiccode == 'P') && (*(char *)((int)&cfg_new->magiccode + 1) == 'C'))
        && (*(char *)((int)&cfg_new->magiccode + 2) == 'F')) &&
       (*(char *)((int)&cfg_new->magiccode + 3) == 'G')) {
      uVar1 = cfg_new->crc32;
      uVar2 = BFLB_Soft_CRC32(&cfg_new->cfg,8);
      iVar3 = 0;
      if ((uVar1 == uVar2) && (xtalType = (cfg_new->cfg).xtal_type, xtalType != GLB_PLL_XTAL_NONE))
      {
        GLB_Set_System_CLK(xtalType,(cfg_new->cfg).pll_clk);
        GLB_Set_System_CLK_Div((cfg_new->cfg).hclk_div,(cfg_new->cfg).bclk_div);
        GLB_Set_SF_CLK('\x01',(cfg_new->cfg).flash_clk_type,(cfg_new->cfg).flash_clk_div);
        GLB_Set_UART_CLK('\x01',HBN_UART_CLK_FCLK,'\0');
        bflb_platform_init(0);
        bflb_eflash_loader_clk_set_from_user::clk_locked = lock;
        goto LAB_2201295a;
      }
    }
    iVar3 = 0xffff;
    bflb_eflash_loader_clk_set_from_user::clk_locked = lock;
  }
LAB_2201295a:
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return iVar3;
}



int32_t bflb_eflash_loader_parse_bootheader(uint8_t *data)

{
  byte bVar1;
  uint32_t uVar2;
  int32_t iVar3;
  
  if ((((data[0x76] & 1) == 0) || (*(int *)(data + 0xac) != -0x21524111)) &&
     (uVar2 = BFLB_Soft_CRC32(data,0xac), *(uint32_t *)(data + 0xac) != uVar2)) {
    bflb_platform_printf("bootheader crc error\r\n");
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return 0x204;
  }
  iVar3 = 0x202;
  if ((data[0x75] & 4) == 0) {
    if (((boot_cpu_cfg[0].bootenable != 0) && (*data == (uint8_t)boot_cpu_cfg[0].boot_magic)) &&
       ((data[1] == boot_cpu_cfg[0].boot_magic._1_1_ &&
        ((data[2] == boot_cpu_cfg[0].boot_magic._2_1_ &&
         (data[3] == boot_cpu_cfg[0].boot_magic._3_1_)))))) {
      image_cfg[0].entrypoint = 0;
      image_cfg[0].img_valid = '\0';
      bflb_eflash_loader_clk_set_from_user((boot_clk_cfg_t *)(data + 100),'\x01');
      bVar1 = data[0x75];
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      image_cfg[0].no_segment = bVar1 & 1;
      image_cfg[0].cache_select = bVar1 >> 1 & 1;
      image_cfg[0].cache_way_disable = bVar1 >> 4;
      image_cfg[0].hash_ignore = data[0x76] >> 1 & 1;
      image_cfg[0].aes_region_lock = bVar1 >> 3 & 1;
      image_cfg[0].halt_ap = data[0x76] >> 2 & 1;
      image_cfg[0].img_segment_info = *(int *)(data + 0x78);
      image_cfg[0].entrypoint = *(uint32_t *)(data + 0x7c);
      image_cfg[0].img_start = *(undefined4 *)(data + 0x80);
      return (-(uint)(*(int *)(data + 0x78) != 0) & 0xfffffdf9) + 0x207;
    }
    bflb_platform_printf("magic code error\r\n");
    iVar3 = 0x203;
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return iVar3;
}



int32_t bflb_eflash_loader_boot_pre_jump(void)

{
  bool bVar1;
  int iStack20;
  uint32_t i;
  
  GLB_AHB_Slave1_Reset(BL_AHB_SLAVE1_SEC);
  bflb_platform_deinit();
  bflb_eflash_loader_interface_deinit();
  if ((image_cfg[0].img_valid != '\0') && (image_cfg[0].entrypoint != 0)) {
    (*(code *)image_cfg[0].entrypoint)();
  }
  do {
    iStack20 = 600;
    do {
      bVar1 = iStack20 != 0;
      iStack20 = iStack20 + -1;
    } while (bVar1);
  } while( true );
}



int32_t bflb_eflash_loader_boot_main(void)

{
  int32_t iVar1;
  
  image_cfg[0].mspval = 0;
  image_cfg[0].entrypoint = 0;
  image_cfg[0]._8_4_ = image_cfg[0]._8_4_ & 0xffff00ff;
  iVar1 = bflb_spi_flash_read(boot_cpu_cfg[0].flash_boot_offset,(uint32_t *)eflash_loader_readbuf,
                              0xb0);
  if ((iVar1 == 0) &&
     (iVar1 = bflb_eflash_loader_parse_bootheader((uint8_t *)eflash_loader_readbuf), iVar1 == 0)) {
    image_cfg[0]._0_4_ =
         image_cfg[0]._0_4_ & 0xffffff | (uint)((image_cfg[0]._4_4_ & 0xff) != 0) << 0x18;
    if ((image_cfg[0]._4_4_ & 0xff) == 0) goto LAB_22012c8e;
    if (image_cfg[0].entrypoint == 0) goto LAB_22012ca8;
    goto LAB_22012c8e;
  }
  bflb_platform_printf("CPU %d boot fail\r\n",0);
  image_cfg[0]._0_4_ = 0;
  image_cfg[0]._4_4_ = 0;
  image_cfg[0]._8_4_ = 0;
  image_cfg[0].img_segment_info = 0;
  image_cfg[0].mspval = 0;
  image_cfg[0].entrypoint = 0;
  image_cfg[0].img_start = 0;
  image_cfg[0].sig_len = 0;
  image_cfg[0].sig_len2 = 0;
  image_cfg[0].deallen = 0;
  image_cfg[0].maxinputlen = 0;
  bflb_spi_flash_read(0,(uint32_t *)eflash_loader_readbuf,4);
  if ((eflash_loader_readbuf[0][0] != 0x504e4642) &&
     ((otp_cfg.hw_cfg.soft_dbg_config >> 4 & 1) != 0)) {
    do {
      image_cfg[0]._0_4_ = CONCAT13(1,image_cfg[0]._0_3_);
      image_cfg[0].img_start = 0;
      image_cfg[0]._4_3_ = CONCAT12(0xf,CONCAT11(1,image_cfg[0].no_segment));
      image_cfg[0]._4_4_ = image_cfg[0]._4_4_ & 0xff000000 | (uint)image_cfg[0]._4_3_;
LAB_22012ca8:
      image_cfg[0].entrypoint = 0x23000000;
LAB_22012c8e:
      bflb_eflash_loader_cache_enable('\x01');
      bflb_eflash_loader_boot_pre_jump();
    } while( true );
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0x21b;
}



int32_t bflb_eflash_loader_cache_enable(uint8_t cont_read)

{
  BL_Err_Type BVar1;
  undefined3 in_register_00002029;
  undefined3 extraout_var;
  uint8_t auStack20 [4];
  uint32_t tmp [1];
  
  if (CONCAT31(in_register_00002029,cont_read) == 1) {
    SF_Ctrl_Set_Owner(SF_CTRL_OWNER_SAHB);
    BVar1 = SFlash_Read(&flashCfg_Gd_Q80E_Q16E,flashCfg_Gd_Q80E_Q16E.ioMode & 0xf,'\x01',0,auStack20
                        ,4);
    if (CONCAT31(extraout_var,BVar1) != 0) {
      bflb_platform_printf("read fail %d\r\n",CONCAT31(extraout_var,BVar1));
    }
  }
  if (image_cfg[0].cache_select == '\0') {
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return 0;
  }
  SF_Ctrl_Set_Flash_Image_Offset(image_cfg[0].img_start);
  SFlash_Cache_Read_Enable
            (&flashCfg_Gd_Q80E_Q16E,flashCfg_Gd_Q80E_Q16E.ioMode & 0xf,cont_read,
             image_cfg[0].cache_way_disable);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t bflb_spi_flash_update_para(uint8_t *data)

{
  BL602_MemCpy_Fast(&flashCfg_Gd_Q80E_Q16E,data,0x54);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t bflb_spi_flash_xip_read_init(void)

{
  BL_Err_Type BVar1;
  undefined3 extraout_var;
  undefined3 extraout_var_00;
  SF_Ctrl_IO_Type ioMode;
  uint8_t auStack20 [4];
  uint32_t tmp [1];
  int iVar2;
  
  if ((flashCfg_Gd_Q80E_Q16E.ioMode >> 4 & 1) == 0) {
    bflb_platform_printf("burst wrap\r\n");
    L1C_Set_Wrap(ENABLE);
    SFlash_Write_Enable(&flashCfg_Gd_Q80E_Q16E);
    ioMode = flashCfg_Gd_Q80E_Q16E.ioMode & 0xf;
    if ((ioMode + ~SF_CTRL_DO_MODE & 0xfd) == 0) {
      SFlash_SetBurstWrap(&flashCfg_Gd_Q80E_Q16E);
      BVar1 = SFlash_Read(&flashCfg_Gd_Q80E_Q16E,flashCfg_Gd_Q80E_Q16E.ioMode & 0xf,'\x01',0,
                          auStack20,4);
      iVar2 = CONCAT31(extraout_var_00,BVar1);
      goto joined_r0x22012e10;
    }
  }
  else {
    bflb_platform_printf("unwrap\r\n");
    L1C_Set_Wrap(DISABLE);
    ioMode = flashCfg_Gd_Q80E_Q16E.ioMode & 0xf;
  }
  BVar1 = SFlash_Read(&flashCfg_Gd_Q80E_Q16E,ioMode,'\x01',0,auStack20,4);
  iVar2 = CONCAT31(extraout_var,BVar1);
joined_r0x22012e10:
  if (iVar2 != 0) {
    bflb_platform_printf("XI fail\r\n");
  }
  SF_Ctrl_Set_Flash_Image_Offset(0);
  SFlash_Cache_Read_Enable(&flashCfg_Gd_Q80E_Q16E,flashCfg_Gd_Q80E_Q16E.ioMode & 0xf,'\x01','\x0f');
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t bflb_spi_flash_xip_read_exit(void)

{
  SF_Ctrl_Set_Owner(SF_CTRL_OWNER_SAHB);
  SFlash_Reset_Continue_Read(&flashCfg_Gd_Q80E_Q16E);
  SFlash_DisableBurstWrap(&flashCfg_Gd_Q80E_Q16E);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



void bflb_spi_flash_set_io_mode(uint8_t ioMode)

{
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  flashCfg_Gd_Q80E_Q16E.ioMode = ioMode;
  return;
}



void bflb_spi_flash_read_jedec_id(uint8_t *data)

{
  uint uStack20;
  uint32_t jid;
  
  uStack20 = 0;
  SFlash_GetJedecId(&flashCfg_Gd_Q80E_Q16E,(uint8_t *)&uStack20);
  uStack20 = uStack20 & 0xffffff;
  if (bflb_eflash_loader_flash_id_valid_flag != 0) {
    uStack20 = uStack20 | 0x80000000;
  }
  BL602_MemCpy(data,&uStack20,4);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



BL_Err_Type
bflb_spi_flash_read_status_reg_with_cmd(uint8_t readRegCmd,uint8_t *regValue,uint8_t regLen)

{
  BL_Err_Type BVar1;
  
  BVar1 = SFlash_Read_Reg_With_Cmd(&flashCfg_Gd_Q80E_Q16E,readRegCmd,regValue,regLen);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



BL_Err_Type
bflb_spi_flash_write_status_reg_with_cmd(uint8_t writeRegCmd,uint8_t *regValue,uint8_t regLen)

{
  BL_Err_Type BVar1;
  undefined3 extraout_var;
  
  BVar1 = SFlash_Write_Enable(&flashCfg_Gd_Q80E_Q16E);
  if (CONCAT31(extraout_var,BVar1) != 0) {
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return ERROR;
  }
  BVar1 = SFlash_Write_Reg_With_Cmd(&flashCfg_Gd_Q80E_Q16E,writeRegCmd,regValue,regLen);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



BL_Err_Type bflb_flash_erase(SPI_Flash_Cfg_Type *flashCfg,uint32_t startaddr,uint32_t endaddr)

{
  BL_Err_Type BVar1;
  undefined3 extraout_var;
  undefined3 extraout_var_00;
  undefined3 extraout_var_01;
  uint uVar3;
  uint uVar4;
  int iVar5;
  int iVar2;
  
  do {
    if (endaddr < startaddr) {
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return SUCCESS;
    }
    bflb_eflash_loader_if_send_pending();
    uVar4 = (endaddr + 1) - startaddr;
    if (flashCfg->blk64EraseCmd == 0xff) {
      uVar3 = (uint)flashCfg->sectorSize;
LAB_22012f3c:
      if ((flashCfg->blk32EraseCmd == 0xff) ||
         (((startaddr & 0x7fff) != 0 || (uVar4 <= (0x20 - uVar3) * 0x400)))) {
        startaddr = startaddr & uVar3 * -0x400;
        BVar1 = SFlash_Sector_Erase(flashCfg,startaddr / uVar3 >> 10);
        iVar2 = CONCAT31(extraout_var,BVar1);
        iVar5 = (uint)flashCfg->sectorSize << 10;
      }
      else {
        BVar1 = SFlash_Blk32_Erase(flashCfg,startaddr >> 0xf);
        iVar2 = CONCAT31(extraout_var_00,BVar1);
        iVar5 = 0x8000;
      }
    }
    else {
      uVar3 = (uint)flashCfg->sectorSize;
      if (((startaddr & 0xffff) != 0) || (uVar4 <= (0x40 - uVar3) * 0x400)) goto LAB_22012f3c;
      BVar1 = SFlash_Blk64_Erase(flashCfg,startaddr >> 0x10);
      iVar2 = CONCAT31(extraout_var_01,BVar1);
      iVar5 = 0x10000;
    }
    startaddr = startaddr + iVar5;
    if (iVar2 != 0) {
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return ERROR;
    }
  } while( true );
}



int32_t bflb_spi_flash_erase(uint32_t startaddr,uint32_t endaddr)

{
  BL_Err_Type BVar1;
  undefined3 extraout_var;
  
  BVar1 = bflb_flash_erase(&flashCfg_Gd_Q80E_Q16E,startaddr,endaddr);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return (-(uint)(CONCAT31(extraout_var,BVar1) == 0) & 0xfffffffd) + 3;
}



int32_t bflb_spi_flash_chiperase(void)

{
  BL_Err_Type BVar1;
  undefined3 extraout_var;
  
  BVar1 = bflb_flash_chip_erase(&flashCfg_Gd_Q80E_Q16E);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return (-(uint)(CONCAT31(extraout_var,BVar1) == 0) & 0xfffffffd) + 3;
}



int32_t bflb_spi_flash_program(uint32_t addr,uint8_t *data,uint32_t len)

{
  BL_Err_Type BVar1;
  undefined3 extraout_var;
  
  BVar1 = SFlash_Program(&flashCfg_Gd_Q80E_Q16E,SF_CTRL_NIO_MODE,addr,data,len);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return (-(uint)(CONCAT31(extraout_var,BVar1) == 0) & 0xfffffffd) + 3;
}



int32_t bflb_spi_flash_read_sahb(uint32_t addr,uint32_t *data,uint32_t len)

{
  SFlash_Read(&flashCfg_Gd_Q80E_Q16E,flashCfg_Gd_Q80E_Q16E.ioMode & 0xf,'\0',addr,(uint8_t *)data,
              len);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t bflb_spi_flash_read_xip(uint32_t addr,uint32_t *data,uint32_t len)

{
  uint32_t uVar1;
  
  uVar1 = SF_Ctrl_Get_Flash_Image_Offset();
  BL602_MemCpy_Fast(data,(void *)((addr + 0x23000000) - uVar1),len);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t bflb_spi_flash_read(uint32_t addr,uint32_t *data,uint32_t len)

{
  SFlash_Read(&flashCfg_Gd_Q80E_Q16E,SF_CTRL_NIO_MODE,'\0',addr,(uint8_t *)data,len);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



BL_Err_Type bflb_clear_flash_status_register(SPI_Flash_Cfg_Type *flashCfg)

{
  BL_Err_Type BVar1;
  undefined3 extraout_var;
  undefined3 extraout_var_00;
  int iStack20;
  uint32_t regValue;
  
  iStack20 = 0;
  BVar1 = SFlash_Write_Enable(flashCfg);
  if (CONCAT31(extraout_var,BVar1) == 0) {
    iStack20 = 0;
    if (flashCfg->qeWriteRegLen == '\x02') {
      iStack20 = 1 << ((uint)flashCfg->qeIndex * 8 + (uint)flashCfg->qeBit & 0x1f);
      SFlash_Write_Reg(flashCfg,'\0',(uint8_t *)&iStack20,'\x02');
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return BVar1;
    }
    if (flashCfg->qeIndex == 0) {
      iStack20 = 1 << (flashCfg->qeBit & 0x1f);
    }
    SFlash_Write_Reg(flashCfg,'\0',(uint8_t *)&iStack20,'\x01');
    BVar1 = SFlash_Write_Enable(flashCfg);
    if (CONCAT31(extraout_var_00,BVar1) == 0) {
      iStack20 = 0;
      if (flashCfg->qeIndex == '\x01') {
        iStack20 = 1 << (flashCfg->qeBit & 0x1f);
      }
      SFlash_Write_Reg(flashCfg,'\x01',(uint8_t *)&iStack20,'\x01');
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return BVar1;
    }
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return ERROR;
}



int32_t bflb_spi_flash_init(uint32_t autoScan,uint32_t flashCfg,uint8_t restoreDefault)

{
  BL_Err_Type BVar1;
  uint32_t uVar2;
  undefined3 extraout_var;
  undefined3 extraout_var_00;
  undefined3 extraout_var_01;
  uint8_t auStack33 [4];
  uint8_t aesEnable;
  
  auStack33[0] = '\0';
  XIP_SFlash_Opt_Enter(auStack33);
  SFlash_Releae_Powerdown(&flashCfg_Gd_Q80E_Q16E);
  bflb_eflash_loader_flash_id_valid_flag = 0;
  uVar2 = SF_Cfg_Flash_Identify_Ext('\0',autoScan,flashCfg,restoreDefault,&flashCfg_Gd_Q80E_Q16E);
  if ((int)uVar2 < 0) {
    bflb_eflash_loader_flash_id_valid_flag = 1;
  }
  XIP_SFlash_Opt_Exit(auStack33[0]);
  BVar1 = bflb_clear_flash_status_register(&flashCfg_Gd_Q80E_Q16E);
  if (CONCAT31(extraout_var,BVar1) != 0) {
    bflb_platform_printf("crf\n");
  }
  if ((flashCfg_Gd_Q80E_Q16E.mid == 0xef) &&
     (BVar1 = SFlash_RCV_Enable(&flashCfg_Gd_Q80E_Q16E,'\x15','\x11','\x03'),
     CONCAT31(extraout_var_01,BVar1) != 0)) {
    bflb_platform_printf("rvf\n");
  }
  if ((((flashCfg_Gd_Q80E_Q16E.ioMode & 0xf) - 2 & 0xfd) == 0) &&
     (BVar1 = SFlash_Qspi_Enable(&flashCfg_Gd_Q80E_Q16E), CONCAT31(extraout_var_00,BVar1) != 0)) {
    bflb_platform_printf("qef\n");
  }
  if (-1 < (int)uVar2) {
    flashCfg_Gd_Q80E_Q16E.ioMode = '\0';
    bflb_platform_printf("FD fail\n");
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return 1;
  }
  flashCfg_Gd_Q80E_Q16E.ioMode = '\0';
  bflb_platform_printf("Flash detect:%x\n",uVar2);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



void bflb_efuse_write(uint32_t index,uint32_t *data,uint32_t len)

{
  uint uVar1;
  BL_Sts_Type BVar2;
  undefined3 extraout_var;
  
  if (len < 8) {
    EF_Ctrl_Program_Direct_R0(index >> 2,data + 1,len >> 2);
  }
  else {
    uVar1 = len - 4;
    EF_Ctrl_Program_Direct_R0(index >> 2,data + 1,uVar1 >> 2);
    EF_Ctrl_Program_Direct_R0
              (index + uVar1 >> 2,(uint32_t *)((uVar1 & 0xfffffffc) + 4 + (int)data),1);
  }
  do {
    BVar2 = EF_Ctrl_Busy();
  } while (CONCAT31(extraout_var,BVar2) == 1);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



void bflb_efuse_read(uint32_t index,uint32_t *data,uint32_t len)

{
  EF_Ctrl_Load_Efuse_R0();
  EF_Ctrl_Read_Direct_R0(index >> 2,data,len >> 2);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



void bflb_read_otp(boot_otp_cfg_t *pOtpCfg)

{
  int iVar1;
  BL_Sts_Type BVar2;
  undefined3 extraout_var;
  EF_Ctrl_Sec_Param_Type EStack20;
  EF_Ctrl_Sec_Param_Type cfg;
  
  iVar1 = 0x15;
  EF_Ctrl_Load_Efuse_R0();
  while( true ) {
    BVar2 = EF_Ctrl_AutoLoad_Done();
    iVar1 = iVar1 + -1;
    if ((CONCAT31(extraout_var,BVar2) == 1) || (iVar1 == 0)) break;
    BL602_Delay_US(1);
  }
  EF_Ctrl_Read_Secure_Boot((EF_Ctrl_Sign_Type *)&otp_cfg,otp_cfg.hw_cfg.encrypted);
  if (otp_cfg.hw_cfg.encrypted == '\x02') {
    otp_cfg.hw_cfg.encrypted = '\x03';
  }
  else if (otp_cfg.hw_cfg.encrypted == '\x03') {
    otp_cfg.hw_cfg.encrypted = '\x02';
  }
  EF_Ctrl_Read_Secure_Cfg(&EStack20);
  otp_cfg.hw_cfg.soft_dbg_config = (EStack20.ef_no_hd_boot_en & 1) * '\x10' + EStack20.ef_dbg_mode;
  otp_cfg.hw_cfg.trim_enable = EF_Ctrl_Get_Trim_Enable();
  EF_Ctrl_Read_Sw_Usage(0,(uint32_t *)&otp_cfg.sw_cfg0);
  EF_Ctrl_Read_Chip_ID(otp_cfg.chip_id);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



void xz_BSP_init(void)

{
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



uint8_t * xz_BSP_get_dict_buffer(void)

{
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return (uint8_t *)0x42037000;
}



uint8_t * xz_BSP_get_work_buffer(void)

{
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return (uint8_t *)0x42047000;
}



uint32_t xz_BSP_get_output_size(void)

{
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0x1000;
}



void bflb_set_high_speed_system_clock(GLB_PLL_XTAL_Type xtal)

{
  GLB_Set_System_CLK(xtal,GLB_SYS_CLK_PLL160M);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



void bflb_set_high_speed_flash_clock(void)

{
  GLB_Set_SF_CLK('\x01',GLB_SFLASH_CLK_80M,'\x01');
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



void bflb_set_low_speed_flash_clock(void)

{
  GLB_Set_SF_CLK('\x01',GLB_SFLASH_CLK_80M,'\x03');
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



void bflb_set_flash_clock_div(uint8_t div)

{
  GLB_Set_SF_CLK('\x01',GLB_SFLASH_CLK_80M,div);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



int32_t sdu_init(void)

{
  GLB_AHB_Slave1_Reset(BL_AHB_SLAVE1_SDU);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return 0;
}



int32_t sdu_host_check(void)

{
  uint32_t uVar1;
  pHidSdio_RegMap_t *ppVar2;
  int32_t iVar3;
  uint32_t (*pauVar4) [4098];
  
  if (DAT_4000d160 == '\0') {
    iVar3 = -1;
  }
  else {
    bflb_platform_printf("SDIO Host write\r\n");
    ppVar2 = SdioFuncReg;
    DAT_4000d160 = '\0';
    *(undefined *)&SdioFuncReg->RdBitMap = 0;
    *(undefined *)((int)&ppVar2->RdBitMap + 1) = 0;
    *(undefined *)&ppVar2->WrBitMap = 0;
    *(undefined *)((int)&ppVar2->WrBitMap + 1) = 0;
    ppVar2->CardIntStatus = '\x01';
    SdioFuncReg->CardIntStatus = '\0';
    ppVar2 = SdioFuncReg;
    if (flag_mport == '\0') {
      uVar1 = SdioFuncReg->Config2;
      *(char *)&SdioFuncReg->Config2 = (char)uVar1;
      *(byte *)((int)&ppVar2->Config2 + 1) = (byte)(uVar1 >> 8) | 0xc;
      *(char *)((int)&ppVar2->Config2 + 2) = (char)(uVar1 >> 0x10);
      *(char *)((int)&ppVar2->Config2 + 3) = (char)(uVar1 >> 0x18);
      ppVar2->Config = ppVar2->Config | 0x10;
    }
    SdioFuncReg->CardIntMask = '\a';
    SdioFuncReg->CardIntMode = '\0';
    clic_disable_interrupt(0x17);
    uVar1 = rx_buf_index;
    ppVar2 = SdioFuncReg;
    eflash_loader_readbuf[rx_buf_index & 1][0] = 0xffffffff;
    ppVar2->WrIdx = '\0';
    ppVar2 = SdioFuncReg;
    pauVar4 = eflash_loader_readbuf[uVar1 & 1];
    *(char *)&SdioFuncReg->SqWriteBase = (char)pauVar4;
    *(char *)((int)&ppVar2->SqWriteBase + 1) = (char)((uint)pauVar4 >> 8);
    *(char *)((int)&ppVar2->SqWriteBase + 2) = (char)((uint)pauVar4 >> 0x10);
    *(char *)((int)&ppVar2->SqWriteBase + 3) = (char)((uint)pauVar4 >> 0x18);
    sdu_rx_buf_attached = sdu_rx_buf_attached | 1;
    *(undefined *)&ppVar2->WrBitMap = 1;
    *(undefined *)((int)&ppVar2->WrBitMap + 1) = 0;
    iVar3 = 0;
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return iVar3;
}



void sdu_deinit(void)

{
  SdioFuncReg->CardIntStatus = ~SdioFuncReg->CardIntStatus | 0x10;
  clic_clear_pending(0x17);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



uint32_t * sdu_receive_data(uint32_t *recv_len)

{
  uint8_t *puVar1;
  ushort uVar2;
  pHidSdio_RegMap_t *ppVar3;
  uint32_t (*pauVar4) [4098];
  uint uVar5;
  uint32_t (*pauVar6) [4098];
  uint32_t uVar7;
  uint uVar8;
  int iVar9;
  
  puVar1 = &SdioFuncReg->HostTransferStatus;
  if (((SdioFuncReg->CardIntStatus & 1) != 0) &&
     (SdioFuncReg->CardIntStatus = ~SdioFuncReg->CardIntStatus | 0x10, (*puVar1 & 4) == 0)) {
    uVar5 = (uint)sdu_receive_data::curr_dnld_port;
    if ((((int)(uint)SdioFuncReg->WrBitMap >> (uVar5 & 0x1f) & 1U) == 0) &&
       (((int)(uint)sdu_rx_buf_attached >> (uVar5 & 0x1f) & 1U) != 0)) {
      pauVar4 = eflash_loader_readbuf[rx_buf_index & 1];
      uVar2 = *(ushort *)((int)eflash_loader_readbuf[rx_buf_index & 1] + 2);
      if (uVar2 < 0x4005) {
        uVar7 = uVar2 + 4;
      }
      else {
        uVar7 = 0;
        pauVar4 = (uint32_t (*) [4098])0x0;
      }
      iVar9 = 1 << (uVar5 & 0x1f);
      *recv_len = uVar7;
      ppVar3 = SdioFuncReg;
      sdu_rx_buf_attached = ~(ushort)iVar9 & sdu_rx_buf_attached;
      uVar8 = rx_buf_index + 1;
      rx_buf_index = uVar8;
      eflash_loader_readbuf[uVar8 & 1][0] = 0xffffffff;
      ppVar3->WrIdx = sdu_receive_data::curr_dnld_port;
      ppVar3 = SdioFuncReg;
      pauVar6 = eflash_loader_readbuf[uVar8 & 1];
      sdu_rx_buf_attached = (ushort)iVar9 | sdu_rx_buf_attached;
      *(char *)&SdioFuncReg->SqWriteBase = (char)pauVar6;
      *(char *)((int)&ppVar3->SqWriteBase + 1) = (char)((uint)pauVar6 >> 8);
      *(char *)((int)&ppVar3->SqWriteBase + 2) = (char)((uint)pauVar6 >> 0x10);
      *(char *)((int)&ppVar3->SqWriteBase + 3) = (char)((uint)pauVar6 >> 0x18);
      *(char *)&ppVar3->WrBitMap = (char)iVar9;
      *(char *)((int)&ppVar3->WrBitMap + 1) = (char)((uint)iVar9 >> 8);
      uVar5 = uVar5 + 1 & 0xff;
      if (uVar5 != 1) {
        gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
        sdu_receive_data::curr_dnld_port = (uint8_t)uVar5;
        return (uint32_t *)pauVar4;
      }
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      sdu_receive_data::curr_dnld_port = '\0';
      return (uint32_t *)pauVar4;
    }
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return (uint32_t *)0x0;
}



int32_t sdu_send_data(uint32_t *data,uint32_t len)

{
  pHidSdio_RegMap_t *ppVar1;
  int iVar2;
  uint uVar3;
  uint64_t uVar4;
  
  SdioFuncReg->CardIntStatus = ~SdioFuncReg->CardIntStatus | 0x10;
  uVar3 = (uint)sdu_send_data::curr_upld_port;
  if (((int)(uint)SdioFuncReg->RdBitMap >> (uVar3 & 0x1f) & 1U) != 0) {
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return -1;
  }
  SdioFuncReg->RdIdx = sdu_send_data::curr_upld_port;
  ppVar1 = SdioFuncReg;
  iVar2 = 1 << (uVar3 & 0x1f);
  *(char *)((int)SdioFuncReg->RdLen + uVar3 * 2 + 1) = (char)(len >> 8);
  *(char *)(ppVar1->RdLen + uVar3) = (char)len;
  *(char *)&ppVar1->SqReadBase = (char)data;
  *(char *)((int)&ppVar1->SqReadBase + 1) = (char)((uint)data >> 8);
  *(char *)((int)&ppVar1->SqReadBase + 2) = (char)((uint)data >> 0x10);
  *(char *)((int)&ppVar1->SqReadBase + 3) = (char)((uint)data >> 0x18);
  *(char *)&ppVar1->RdBitMap = (char)iVar2;
  *(char *)((int)&ppVar1->RdBitMap + 1) = (char)((uint)iVar2 >> 8);
  bflb_platform_clear_time();
  do {
    if (((int)(uint)SdioFuncReg->RdBitMap >> (sdu_send_data::curr_upld_port & 0x1f) & 1U) == 0) {
      uVar3 = sdu_send_data::curr_upld_port + 1 & 0xff;
      if (uVar3 != 1) {
        sdu_send_data::curr_upld_port = (uint8_t)uVar3;
      }
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return len;
    }
    uVar4 = bflb_platform_get_time_ms();
  } while ((ppVar1 == (pHidSdio_RegMap_t *)0x0) && (data < (uint32_t *)0x7d1));
  bflb_platform_printf("sdu send timeout\r\n",(int)uVar4);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return -1;
}



void * memcpy(void *dst,void *src,size_t n)

{
  undefined uVar1;
  undefined *puVar2;
  undefined *puVar3;
  
  if (n != 0) {
    puVar2 = (undefined *)(n + (int)src);
    puVar3 = (undefined *)dst;
    do {
                    // WARNING: Load size is inaccurate
      uVar1 = *src;
      src = (void *)((int)src + 1);
      *puVar3 = uVar1;
      puVar3 = puVar3 + 1;
    } while ((undefined *)src != puVar2);
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return dst;
}



void * memset(void *dst,int c,size_t n)

{
  undefined *puVar1;
  undefined *puVar2;
  
  puVar2 = (undefined *)dst;
  if (n != 0) {
    do {
      puVar1 = puVar2 + 1;
      *puVar2 = (char)c;
      puVar2 = puVar1;
    } while (puVar1 != (undefined *)((int)dst + n));
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return dst;
}



int memcmp(void *s1,void *s2,size_t n)

{
  byte bVar1;
  byte bVar2;
  byte *pbVar3;
  
  pbVar3 = (byte *)(n + (int)s2);
  do {
    if ((byte *)s2 == pbVar3) {
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return 0;
    }
                    // WARNING: Load size is inaccurate
    bVar1 = *s2;
                    // WARNING: Load size is inaccurate
    bVar2 = *s1;
    s2 = (void *)((int)s2 + 1);
    s1 = (byte *)((int)s1 + 1);
  } while ((uint)bVar2 == (uint)bVar1);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return (uint)bVar2 - (uint)bVar1;
}



// WARNING: Type propagation algorithm not settling

int vsnprintf(char *buffer,size_t n,char *format,va_list ap)

{
  bool bVar1;
  bool bVar2;
  bool bVar3;
  bool bVar4;
  bool bVar5;
  byte *pbVar6;
  byte *pbVar7;
  byte *pbVar8;
  uint uVar9;
  byte *pbVar10;
  byte *pbVar11;
  int iVar12;
  byte *pbVar13;
  byte bVar14;
  uint uVar15;
  byte *pbVar16;
  byte *pbVar17;
  byte **ppbVar18;
  uint uVar19;
  byte *pbVar20;
  byte *pbVar21;
  byte *pbVar22;
  byte *pbVar23;
  byte *pbVar24;
  byte *pbVar25;
  byte *pbVar26;
  UDItype in_fa1;
  UDItype in_fa2;
  byte **ppbStack140;
  byte *pbStack136;
  byte *pbStack120;
  byte *pbStack96;
  char *pcStack92;
  char *pcStack88;
  byte *pbStack84;
  byte abStack65 [4];
  char carg;
  
  bVar14 = *format;
  pbVar20 = (byte *)buffer;
  if (bVar14 == 0) {
    pbVar26 = (byte *)0x0;
LAB_22013874:
    if (pbVar26 < n) {
      *pbVar20 = 0;
    }
    else if (n != 0) {
      buffer[n - 1] = '\0';
    }
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return (int)pbVar26;
  }
  pbVar26 = (byte *)0x0;
  pcStack92 = "(null)";
  pbVar11 = (byte *)(format + 1);
  ppbStack140 = (byte **)ap;
LAB_22013858:
  do {
    if (bVar14 == 0x25) {
      bVar14 = *pbVar11;
      if (bVar14 != 0) {
        uVar19 = 0;
        bVar5 = false;
        bVar4 = false;
        bVar3 = false;
        pbVar25 = pbVar11 + 2;
        do {
          if (bVar14 == 0x2b) {
            bVar14 = pbVar25[-1];
            uVar19 = uVar19 | 4;
          }
          else if (bVar14 < 0x2c) {
            if (bVar14 == 0x23) {
              bVar14 = pbVar25[-1];
              bVar5 = true;
            }
            else if (bVar14 == 0x27) {
              bVar14 = pbVar25[-1];
              bVar3 = true;
            }
            else {
              if (bVar14 != 0x20) goto LAB_22013936;
              bVar14 = pbVar25[-1];
              uVar19 = uVar19 | 0x10;
            }
          }
          else if (bVar14 == 0x2d) {
            bVar14 = pbVar25[-1];
            uVar19 = uVar19 | 2;
          }
          else {
            if (bVar14 != 0x30) goto LAB_22013936;
            bVar14 = pbVar25[-1];
            uVar19 = uVar19 | 1;
          }
          if (bVar14 == 0) break;
          pbVar11 = pbVar25 + -1;
          pbVar25 = pbVar25 + 1;
        } while( true );
      }
      break;
    }
    if (pbVar26 < n) {
      *pbVar20 = bVar14;
      pbVar20 = pbVar20 + 1;
    }
    bVar14 = *pbVar11;
    pbVar26 = pbVar26 + 1;
    pbVar11 = pbVar11 + 1;
  } while (bVar14 != 0);
  goto LAB_22013874;
LAB_22013936:
  uVar15 = (uint)*pbVar11;
  if (uVar15 != 0) {
    pbVar25 = (byte *)0x0;
    iVar12 = 0;
    uVar9 = uVar15 - 0x30;
    pbVar11 = pbVar11 + 2;
    if (9 < (uVar9 & 0xff)) goto LAB_2201398a;
LAB_22013960:
    uVar15 = (uint)pbVar11[-1];
    pbVar25 = (byte *)(uVar9 + iVar12 * 2);
    while (uVar15 != 0) {
      while( true ) {
        pbVar11 = pbVar11 + 1;
        uVar9 = uVar15 - 0x30;
        iVar12 = (int)pbVar25 * 5;
        if ((uVar9 & 0xff) < 10) goto LAB_22013960;
LAB_2201398a:
        pbVar7 = pbVar11 + -1;
        if (uVar15 != 0x2a) {
          if (uVar15 != 0x2e) {
            bVar14 = pbVar11[-2];
            if (bVar14 == 0) goto LAB_22013874;
            pbVar23 = (byte *)0xffffffff;
            goto LAB_220139a4;
          }
          uVar15 = (uint)*pbVar7;
          if (uVar15 == 0) goto LAB_22013874;
          pbVar23 = (byte *)0x0;
          iVar12 = 0;
          goto joined_r0x22013c34;
        }
        uVar15 = (uint)pbVar11[-1];
        pbVar25 = *ppbStack140;
        ppbStack140 = ppbStack140 + 1;
        if ((int)pbVar25 < 0) break;
        if (uVar15 == 0) goto LAB_22013874;
      }
      pbVar25 = (byte *)-(int)pbVar25;
      uVar19 = uVar19 | 2;
    }
  }
  goto LAB_22013874;
joined_r0x22013c34:
  pbVar11 = pbVar11 + 1;
  if ((uVar15 - 0x30 & 0xff) < 10) {
    bVar14 = pbVar11[-1];
    pbVar23 = (byte *)((uVar15 - 0x30) + iVar12 * 2);
  }
  else {
    if (uVar15 != 0x2a) goto code_r0x22013c66;
    pbVar23 = *ppbStack140;
    ppbStack140 = ppbStack140 + 1;
    if ((int)pbVar23 < 0) {
      pbVar23 = (byte *)0xffffffff;
    }
    bVar14 = pbVar11[-1];
  }
  uVar15 = (uint)bVar14;
  if (uVar15 == 0) goto LAB_22013874;
  iVar12 = (int)pbVar23 * 5;
  goto joined_r0x22013c34;
code_r0x22013c66:
  bVar14 = pbVar11[-2];
  pbVar7 = pbVar11 + -1;
  if (bVar14 == 0) goto LAB_22013874;
LAB_220139a4:
  iVar12 = 0;
joined_r0x220139c6:
  pbVar11 = pbVar7 + 1;
  if (bVar14 == 0x6c) {
    bVar14 = *pbVar7;
    iVar12 = iVar12 + 1;
joined_r0x220139f0:
    pbVar7 = pbVar11;
    if (bVar14 == 0) goto LAB_22013874;
    goto joined_r0x220139c6;
  }
  if (0x6c < bVar14) {
    if ((bVar14 != 0x74) && (bVar14 != 0x7a)) {
      if (bVar14 == 0x71) goto LAB_220139fc;
      if (2 < iVar12) {
        iVar12 = 2;
      }
      if (iVar12 < -2) {
        iVar12 = -2;
      }
      if (bVar14 == 0x6f) {
        bVar4 = false;
        pbVar24 = (byte *)0x8;
        goto is_unsigned;
      }
      if (bVar14 < 0x70) goto LAB_22013b54;
      if (bVar14 == 0x75) {
        bVar4 = false;
        pbVar24 = (byte *)0xa;
        goto is_unsigned;
      }
      if (0x75 < bVar14) {
        bVar4 = false;
        pbVar24 = (byte *)0x10;
        if (bVar14 != 0x78) goto LAB_22013b6c;
        goto is_unsigned;
      }
      if (bVar14 == 0x70) goto LAB_22013fa0;
      if (bVar14 != 0x73) goto LAB_22013b6c;
      pbVar24 = *ppbStack140;
      if (pbVar24 == (byte *)0x0) {
        pbVar16 = (byte *)0x6;
        pbVar24 = (byte *)pcStack92;
      }
      else {
        pbVar16 = (byte *)strlen((char *)pbVar24);
      }
      goto is_string;
    }
    bVar14 = *pbVar7;
    if (bVar14 == 0) goto LAB_22013874;
    iVar12 = 1;
    pbVar7 = pbVar11;
    goto joined_r0x220139c6;
  }
  if (bVar14 == 0x68) {
    bVar14 = *pbVar7;
    iVar12 = iVar12 + -1;
    goto joined_r0x220139f0;
  }
  if (bVar14 == 0x6a) {
    bVar14 = *pbVar7;
    if (bVar14 == 0) goto LAB_22013874;
    iVar12 = 2;
    pbVar7 = pbVar11;
    goto joined_r0x220139c6;
  }
  if (bVar14 == 0x4c) {
LAB_220139fc:
    bVar14 = *pbVar7;
    iVar12 = iVar12 + 2;
    goto joined_r0x220139f0;
  }
  if (2 < iVar12) {
    iVar12 = 2;
  }
  if (iVar12 < -2) {
    iVar12 = -2;
  }
  if (bVar14 == 99) {
    pbVar16 = (byte *)0x1;
    abStack65[0] = (byte)*ppbStack140;
    pbVar24 = abStack65;
is_string:
    ppbStack140 = ppbStack140 + 1;
    if ((pbVar23 != (byte *)0xffffffff) && ((int)pbVar23 < (int)pbVar16)) {
      pbVar16 = pbVar23;
    }
    if (((int)pbVar16 < (int)pbVar25) && ((uVar19 & 2) == 0)) {
      pbVar23 = pbVar25 + (int)pbVar26;
      do {
        if (pbVar26 < n) {
          *pbVar20 = (char)((uVar19 & 1) << 4) + 0x20;
          pbVar20 = pbVar20 + 1;
        }
        pbVar26 = pbVar26 + 1;
        pbVar25 = pbVar16;
      } while (pbVar26 != pbVar23 + -(int)pbVar16);
    }
    if (pbVar16 != (byte *)0x0) {
      pbVar23 = pbVar24;
      pbVar17 = pbVar20;
      do {
        pbVar21 = pbVar23 + 1;
        pbVar20 = pbVar17;
        if (pbVar26 + ((int)pbVar23 - (int)pbVar24) < n) {
          pbVar20 = pbVar17 + 1;
          *pbVar17 = *pbVar23;
        }
        pbVar23 = pbVar21;
        pbVar17 = pbVar20;
      } while (pbVar24 + (int)pbVar16 != pbVar21);
      pbVar26 = pbVar26 + (int)pbVar16;
    }
    if (((int)pbVar16 < (int)pbVar25) && ((uVar19 & 2) != 0)) {
      pbVar23 = pbVar26;
      do {
        if (pbVar23 < n) {
          *pbVar20 = 0x20;
          pbVar20 = pbVar20 + 1;
        }
        pbVar23 = pbVar23 + 1;
      } while ((int)pbVar16 < (int)(pbVar25 + (int)pbVar26) - (int)pbVar23);
      pbVar23 = (byte *)0x0;
      if ((int)pbVar16 < (int)pbVar25) {
        pbVar23 = pbVar25 + (-1 - (int)pbVar16);
      }
      bVar14 = *pbVar7;
      pbVar26 = pbVar26 + (int)(pbVar23 + 1);
    }
    else {
      bVar14 = *pbVar7;
    }
    goto joined_r0x220140c6;
  }
LAB_22013b54:
  if (bVar14 < 100) {
    if (bVar14 == 0x50) {
      bVar4 = true;
LAB_22013fa0:
      bVar2 = false;
      pbVar17 = *ppbStack140;
      ppbStack140 = ppbStack140 + 1;
      pbVar16 = (byte *)0x0;
      bVar5 = true;
      pbVar24 = (byte *)0x10;
      pbVar23 = (byte *)0x8;
    }
    else {
      if (bVar14 != 0x58) goto LAB_22013b6c;
      bVar4 = true;
      pbVar24 = (byte *)0x10;
is_unsigned:
      bVar2 = false;
      if (iVar12 == 1) {
LAB_22013a7a:
        pbVar17 = *ppbStack140;
        ppbStack140 = ppbStack140 + 1;
        pbVar16 = (byte *)0x0;
      }
      else if (iVar12 == 2) {
        ppbStack140 = (byte **)((uint)((int)ppbStack140 + 7) & 0xfffffff8);
        pbVar17 = *ppbStack140;
        pbVar16 = ppbStack140[1];
        ppbStack140 = ppbStack140 + 2;
      }
      else if (iVar12 == -1) {
        pbVar16 = (byte *)0x0;
        pbVar17 = (byte *)(uint)*(ushort *)ppbStack140;
        ppbStack140 = ppbStack140 + 1;
      }
      else {
        if (iVar12 == 0) goto LAB_22013a7a;
        pbVar17 = (byte *)(uint)*(byte *)ppbStack140;
        ppbStack140 = ppbStack140 + 1;
        pbVar16 = (byte *)0x0;
      }
    }
  }
  else {
    if (bVar14 == 0x6e) {
      ppbVar18 = (byte **)*ppbStack140;
      ppbStack140 = ppbStack140 + 1;
      if (iVar12 != 1) {
        if (iVar12 == 2) {
          *ppbVar18 = pbVar26;
          ppbVar18[1] = (byte *)0x0;
          bVar14 = *pbVar7;
          goto joined_r0x220140c6;
        }
        if (iVar12 == -1) {
          *(short *)ppbVar18 = (short)pbVar26;
          bVar14 = *pbVar7;
          goto joined_r0x220140c6;
        }
        if (iVar12 != 0) {
          *(char *)ppbVar18 = (char)pbVar26;
          bVar14 = *pbVar7;
          goto joined_r0x220140c6;
        }
      }
      *ppbVar18 = pbVar26;
      bVar14 = *pbVar7;
      goto joined_r0x220140c6;
    }
    if ((bVar14 == 0x6f) || ((bVar14 != 100 && (bVar14 != 0x69)))) {
LAB_22013b6c:
      if (pbVar26 < n) {
        *pbVar20 = bVar14;
        pbVar20 = pbVar20 + 1;
      }
      bVar14 = *pbVar7;
      pbVar26 = pbVar26 + 1;
      goto joined_r0x220140c6;
    }
    bVar4 = false;
    if (iVar12 == 1) {
LAB_22013bbe:
      pbVar17 = *ppbStack140;
    }
    else {
      if (iVar12 == 2) {
        bVar2 = true;
        ppbVar18 = (byte **)((uint)((int)ppbStack140 + 7) & 0xfffffff8);
        ppbStack140 = ppbVar18 + 2;
        pbVar17 = *ppbVar18;
        pbVar16 = ppbVar18[1];
        pbVar24 = (byte *)0xa;
        goto is_integer;
      }
      if (iVar12 == -1) {
        pbVar17 = (byte *)(int)*(short *)ppbStack140;
      }
      else {
        if (iVar12 == 0) goto LAB_22013bbe;
        pbVar17 = (byte *)(int)*(char *)ppbStack140;
      }
    }
    ppbStack140 = ppbStack140 + 1;
    pbVar16 = (byte *)((int)pbVar17 >> 0x1f);
    bVar2 = true;
    pbVar24 = (byte *)0xa;
  }
is_integer:
  pbVar21 = (byte *)0x0;
  if (pbVar26 < n) {
    pbVar21 = (byte *)(n - (int)pbVar26);
  }
  pcStack88 = "0123456789ABCDEF";
  if (!bVar4) {
    pcStack88 = "0123456789abcdef";
  }
  if ((bVar2) && ((int)pbVar16 < 0)) {
    pbVar16 = (byte *)(-(uint)(pbVar17 != (byte *)0x0) - (int)pbVar16);
    pbVar17 = (byte *)-(int)pbVar17;
    pbStack84 = (byte *)0x1;
    bVar2 = true;
LAB_22013aac:
    pbVar22 = (byte *)0x0;
    pbVar6 = pbVar17;
    pbVar10 = pbVar16;
    do {
      do {
        pbVar8 = pbVar6;
        pbVar6 = pbVar8;
        pbVar13 = pbVar10;
        __udivdi3(in_fa1,in_fa2);
        pbVar22 = pbVar22 + 1;
        bVar1 = pbVar10 != (byte *)0x0;
        pbVar10 = pbVar13;
      } while (bVar1);
    } while (pbVar24 <= pbVar8);
  }
  else {
    pbStack84 = (byte *)0x0;
    bVar2 = false;
    if (((uint)pbVar17 | (uint)pbVar16) != 0) goto LAB_22013aac;
    pbVar22 = (byte *)0x0;
  }
  if ((bVar5) && (pbVar24 == (byte *)0x8)) {
    if ((int)pbVar23 <= (int)pbVar22) {
      pbVar23 = pbVar22 + 1;
      goto LAB_22013ae6;
    }
LAB_22013caa:
    pbVar22 = pbVar23;
    pbVar23 = pbVar22 + -1;
  }
  else {
LAB_22013ae6:
    if ((int)pbVar22 < (int)pbVar23) goto LAB_22013caa;
    if (((uint)pbVar17 | (uint)pbVar16) == 0) {
      pbVar23 = (byte *)0x0;
      pbVar22 = (byte *)0x1;
    }
    else {
      pbVar23 = pbVar22 + -1;
    }
  }
  pbStack96 = pbVar22;
  if (bVar3) {
    pbStack96 = (byte *)((pbVar24 == (byte *)0x10) + 3);
  }
  pbVar22 = pbVar22 + (int)pbVar23 / (int)pbStack96;
  pbStack84 = pbVar22 + ((uVar19 & 0x14 | (uint)pbStack84) != 0);
  if ((bVar5) && (pbVar24 == (byte *)0x10)) {
    pbStack84 = pbStack84 + 2;
  }
  pbStack120 = pbVar20;
  if ((uVar19 & 3) == 0) {
    if ((int)pbStack84 < (int)pbVar25) {
      pbVar23 = pbVar25 + -(int)pbStack84;
      pbVar16 = (byte *)0x0;
      do {
        if (pbVar16 < pbVar21) {
          *pbStack120 = 0x20;
          pbStack120 = pbStack120 + 1;
        }
        pbVar16 = pbVar16 + 1;
        pbVar25 = pbStack84;
      } while (pbVar16 != pbVar23);
    }
    else {
      pbVar23 = (byte *)0x0;
    }
  }
  else {
    pbVar23 = (byte *)0x0;
  }
  if (bVar2) {
    if (pbVar23 < pbVar21) {
      *pbStack120 = 0x2d;
      pbStack120 = pbStack120 + 1;
    }
LAB_22013d22:
    pbVar23 = pbVar23 + 1;
  }
  else {
    if ((uVar19 & 4) != 0) {
      if (pbVar23 < pbVar21) {
        *pbStack120 = 0x2b;
        pbStack120 = pbStack120 + 1;
      }
      goto LAB_22013d22;
    }
    if ((uVar19 & 0x10) != 0) {
      if (pbVar23 < pbVar21) {
        *pbStack120 = 0x20;
        pbStack120 = pbStack120 + 1;
      }
      goto LAB_22013d22;
    }
  }
  if ((bVar5) && (pbVar24 == (byte *)0x10)) {
    pbVar24 = pbStack120;
    if (pbVar23 < pbVar21) {
      *pbStack120 = 0x30;
      pbVar24 = pbStack120 + 1;
    }
    pbStack120 = pbVar24;
    if (pbVar23 + 1 < pbVar21) {
      pbStack120 = pbVar24 + 1;
      *pbVar24 = !bVar4 * ' ' + 0x58;
    }
    pbVar23 = pbVar23 + 2;
  }
  if ((((uVar19 & 3) == 1) && ((int)pbVar22 < (int)pbVar25)) && ((int)pbStack84 < (int)pbVar25)) {
    pbVar24 = pbVar23 + ((int)pbVar25 - (int)pbStack84);
    do {
      if (pbVar23 < pbVar21) {
        *pbStack120 = 0x30;
        pbStack120 = pbStack120 + 1;
      }
      pbVar23 = pbVar23 + 1;
      pbVar25 = pbStack84;
    } while (pbVar23 != pbVar24);
  }
  pbStack120 = pbStack120 + (int)pbVar22;
  pbVar23 = pbVar22 + (int)pbVar23;
  if (0 < (int)pbVar22) {
    pbStack136 = pbStack96;
    pbVar16 = pbVar23;
    pbVar24 = pbStack120;
    do {
      pbVar6 = pbVar24 + -1;
      pbVar13 = pbVar16 + -1;
      pbVar10 = pbVar22 + -1;
      if (pbStack136 == (byte *)0x0) {
        if (pbVar13 < pbVar21) {
          pbVar24[-1] = 0x5f;
        }
        pbVar13 = pbVar16 + -2;
        pbVar10 = pbVar22 + -2;
        pbVar16 = pbVar24 + -2;
        pbVar24 = pbVar6;
        pbVar6 = pbVar16;
        pbStack136 = pbStack96;
      }
      pbStack136 = pbStack136 + -1;
      if (pbVar13 < pbVar21) {
        pbVar16 = pbVar17;
        __umoddi3(in_fa1,in_fa2);
        pbVar24[-1] = pbVar16[(int)pcStack88];
      }
      __udivdi3(in_fa1,in_fa2);
      pbVar16 = pbVar13;
      pbVar24 = pbVar6;
      pbVar22 = pbVar10;
    } while (0 < (int)pbVar10);
  }
  if (((uVar19 & 2) != 0) && ((int)pbStack84 < (int)pbVar25)) {
    pbVar25 = pbVar25 + (int)pbVar23;
    pbVar24 = pbVar23;
    do {
      if (pbVar24 < pbVar21) {
        *pbStack120 = 0x20;
        pbStack120 = pbStack120 + 1;
      }
      pbVar24 = pbVar24 + 1;
      pbVar23 = pbVar25 + -(int)pbStack84;
    } while (pbVar24 != pbVar25 + -(int)pbStack84);
  }
  bVar14 = *pbVar7;
  pbVar20 = pbVar20 + (int)pbVar23;
  pbVar26 = pbVar26 + (int)pbVar23;
joined_r0x220140c6:
  if (bVar14 == 0) goto LAB_22013874;
  goto LAB_22013858;
}



size_t strlen(char *s)

{
  char *pcVar1;
  char *pcVar2;
  
  pcVar2 = s;
  if (*s == '\0') {
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return 0;
  }
  do {
    pcVar1 = pcVar2 + 1;
    pcVar2 = pcVar2 + 1;
  } while (*pcVar1 != '\0');
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return (int)pcVar2 - (int)s;
}



void xz_crc32_init(void)

{
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



uint32_t xz_crc32(uint8_t *buf,size_t size,uint32_t crc)

{
  byte bVar1;
  byte *pbVar2;
  uint uVar3;
  
  uVar3 = ~crc;
  if (size != 0) {
    pbVar2 = buf + size;
    do {
      bVar1 = *buf;
      buf = buf + 1;
      uVar3 = uVar3 >> 8 ^ crc32Tab[(uint)bVar1 ^ uVar3 & 0xff];
    } while (buf != pbVar2);
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return ~uVar3;
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return crc;
}



void lzma_len(xz_dec_lzma2 *s,lzma_len_dec *l,uint32_t pos_state)

{
  ushort uVar1;
  int iVar2;
  uint uVar3;
  size_t sVar4;
  uint uVar5;
  uint uVar6;
  uint uVar7;
  uint uVar8;
  uint16_t *puVar9;
  uint16_t (*pauVar10) [8];
  
  uVar6 = (s->rc).range;
  uVar8 = (s->rc).code;
  if (uVar6 < 0x1000000) {
    sVar4 = (s->rc).in_pos;
    uVar6 = uVar6 << 8;
    (s->rc).range = uVar6;
    (s->rc).in_pos = sVar4 + 1;
    uVar8 = uVar8 * 0x100 + (uint)(s->rc).in[sVar4];
    (s->rc).code = uVar8;
  }
  uVar1 = l->choice;
  uVar7 = (uVar6 >> 0xb) * (uint)uVar1;
  if (uVar8 < uVar7) {
    l->choice = uVar1 + (short)((int)(0x800 - (uint)uVar1) >> 5);
    pauVar10 = l->low[pos_state];
    (s->lzma).len = 2;
    iVar2 = -6;
    uVar6 = 8;
  }
  else {
    uVar8 = uVar8 - uVar7;
    (s->rc).code = uVar8;
    uVar6 = uVar6 - uVar7;
    l->choice = uVar1 - (uVar1 >> 5);
    if (uVar6 < 0x1000000) {
      sVar4 = (s->rc).in_pos;
      uVar6 = uVar6 * 0x100;
      (s->rc).range = uVar6;
      (s->rc).in_pos = sVar4 + 1;
      uVar8 = uVar8 * 0x100 + (uint)(s->rc).in[sVar4];
      (s->rc).code = uVar8;
    }
    uVar1 = l->choice2;
    uVar7 = (uVar6 >> 0xb) * (uint)uVar1;
    if (uVar8 < uVar7) {
      l->choice2 = uVar1 + (short)((int)(0x800 - (uint)uVar1) >> 5);
      pauVar10 = l->mid[pos_state];
      (s->lzma).len = 10;
      iVar2 = 2;
      uVar6 = 8;
    }
    else {
      uVar8 = uVar8 - uVar7;
      (s->rc).code = uVar8;
      l->choice2 = uVar1 - (uVar1 >> 5);
      pauVar10 = (uint16_t (*) [8])l->high;
      (s->lzma).len = 0x12;
      uVar7 = uVar6 - uVar7;
      iVar2 = -0xee;
      uVar6 = 0x100;
    }
  }
  uVar5 = 1;
  while( true ) {
    puVar9 = *pauVar10 + uVar5;
    if (uVar7 < 0x1000000) {
      sVar4 = (s->rc).in_pos;
      uVar7 = uVar7 << 8;
      (s->rc).range = uVar7;
      (s->rc).in_pos = sVar4 + 1;
      uVar8 = (uint)(s->rc).in[sVar4] + uVar8 * 0x100;
      (s->rc).code = uVar8;
    }
    uVar1 = *puVar9;
    uVar3 = (uVar7 >> 0xb) * (uint)uVar1;
    if (uVar8 < uVar3) {
      (s->rc).range = uVar3;
      *puVar9 = (short)((int)(0x800 - (uint)uVar1) >> 5) + uVar1;
      uVar5 = uVar5 * 2;
    }
    else {
      (s->rc).range = uVar7 - uVar3;
      (s->rc).code = uVar8 - uVar3;
      *puVar9 = uVar1 - (uVar1 >> 5);
      uVar5 = uVar5 * 2 + 1;
    }
    if (uVar6 <= uVar5) break;
    uVar7 = (s->rc).range;
    uVar8 = (s->rc).code;
  }
  (s->lzma).len = uVar5 + iVar2;
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



_Bool dict_repeat(dictionary *dict,uint32_t *len,uint32_t dist)

{
  uint uVar1;
  uint uVar2;
  uint uVar3;
  
  uVar1 = dict->pos;
  uVar2 = *len;
  uVar3 = dict->limit - uVar1;
  if (uVar2 < uVar3) {
    uVar3 = uVar2;
  }
  *len = uVar2 - uVar3;
  uVar2 = ~dist + uVar1;
  if (uVar1 <= dist) {
    uVar2 = uVar2 + dict->end;
  }
  do {
    dict->pos = uVar1 + 1;
    dict->buf[uVar1] = dict->buf[uVar2];
    uVar3 = uVar3 - 1;
    uVar1 = dict->pos;
    uVar2 = -(uint)(uVar2 + 1 != dict->end) & uVar2 + 1;
  } while (uVar3 != 0);
  if (dict->full < uVar1) {
    dict->full = uVar1;
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return true;
}



_Bool lzma_main(xz_dec_lzma2 *s)

{
  ushort uVar1;
  uint uVar2;
  _Bool _Var3;
  undefined3 extraout_var;
  uint uVar4;
  uint uVar5;
  size_t sVar6;
  uint uVar7;
  uint32_t uVar8;
  uint uVar9;
  int iVar10;
  int iVar11;
  uint uVar12;
  lzma_state lVar13;
  uint32_t uVar14;
  uint uVar15;
  ushort *puVar16;
  uint8_t *puVar17;
  uint32_t uVar18;
  uint16_t *puVar19;
  uint uVar20;
  
  uVar2 = (s->dict).pos;
  if (uVar2 < (s->dict).limit) {
    if ((s->lzma).len != 0) {
      uVar9 = (s->lzma).rep0;
      if ((uVar9 < (s->dict).full) && (uVar9 < (s->dict).size)) {
        dict_repeat(&s->dict,&(s->lzma).len,uVar9);
        uVar2 = (s->dict).pos;
        if ((s->dict).limit <= uVar2) goto LAB_22014490;
      }
    }
    do {
      uVar9 = (s->rc).in_pos;
      uVar15 = (s->rc).range;
      if ((s->rc).in_limit < uVar9) goto LAB_22014494;
      uVar5 = (uint)(s->lzma).state;
      uVar12 = (s->rc).code;
      uVar7 = uVar2 & (s->lzma).pos_mask;
      if (uVar15 < 0x1000000) {
        uVar15 = uVar15 << 8;
        (s->rc).range = uVar15;
        (s->rc).in_pos = uVar9 + 1;
        uVar12 = uVar12 * 0x100 + (uint)(s->rc).in[uVar9];
        (s->rc).code = uVar12;
      }
      uVar1 = (s->lzma).is_match[uVar5][uVar7];
      uVar9 = (uVar15 >> 0xb) * (uint)uVar1;
      if (uVar12 < uVar9) {
        (s->rc).range = uVar9;
        (s->lzma).is_match[uVar5][uVar7] = uVar1 + (short)((int)(0x800 - (uint)uVar1) >> 5);
        uVar15 = uVar2;
        if (uVar2 == 0) {
          uVar15 = (s->dict).end;
        }
        uVar20 = (s->dict).full;
        puVar17 = (s->dict).buf;
        uVar7 = 0;
        if (uVar20 != 0) {
          uVar7 = (uint)puVar17[uVar15 - 1];
        }
        uVar15 = (s->lzma).lc;
        iVar10 = ((uVar2 & (s->lzma).literal_pos_mask) << (uVar15 & 0x1f)) +
                 (uVar7 >> (8 - uVar15 & 0x1f));
        if (uVar5 < 7) {
          uVar15 = 1;
          while( true ) {
            puVar19 = (s->lzma).literal[iVar10] + uVar15;
            if (uVar9 < 0x1000000) {
              sVar6 = (s->rc).in_pos;
              uVar9 = uVar9 << 8;
              (s->rc).range = uVar9;
              (s->rc).in_pos = sVar6 + 1;
              uVar12 = (uint)(s->rc).in[sVar6] + uVar12 * 0x100;
              (s->rc).code = uVar12;
            }
            uVar1 = *puVar19;
            uVar5 = (uVar9 >> 0xb) * (uint)uVar1;
            if (uVar12 < uVar5) {
              (s->rc).range = uVar5;
              *puVar19 = (short)((int)(0x800 - (uint)uVar1) >> 5) + uVar1;
              uVar15 = uVar15 * 2;
            }
            else {
              (s->rc).range = uVar9 - uVar5;
              (s->rc).code = uVar12 - uVar5;
              *puVar19 = uVar1 - (uVar1 >> 5);
              uVar15 = uVar15 * 2 + 1;
            }
            if (0xff < uVar15) break;
            uVar9 = (s->rc).range;
            uVar12 = (s->rc).code;
          }
        }
        else {
          uVar15 = (s->lzma).rep0;
          iVar11 = ~uVar15 + uVar2;
          if (uVar2 <= uVar15) {
            iVar11 = iVar11 + (s->dict).end;
          }
          if (uVar20 != 0) {
            uVar20 = (uint)puVar17[iVar11] << 1;
          }
          uVar15 = 1;
          uVar5 = 0x100;
          while( true ) {
            uVar7 = uVar20 & uVar5;
            puVar19 = (s->lzma).literal[iVar10] + uVar5 + uVar15 + uVar7;
            uVar20 = uVar20 << 1;
            if (uVar9 < 0x1000000) {
              sVar6 = (s->rc).in_pos;
              uVar9 = uVar9 << 8;
              (s->rc).range = uVar9;
              (s->rc).in_pos = sVar6 + 1;
              uVar12 = (uint)(s->rc).in[sVar6] + uVar12 * 0x100;
              (s->rc).code = uVar12;
            }
            uVar1 = *puVar19;
            uVar4 = (uVar9 >> 0xb) * (uint)uVar1;
            uVar15 = uVar15 * 2;
            if (uVar12 < uVar4) {
              (s->rc).range = uVar4;
              *puVar19 = (short)((int)(0x800 - (uint)uVar1) >> 5) + uVar1;
              uVar7 = uVar5 & ~uVar7;
            }
            else {
              (s->rc).range = uVar9 - uVar4;
              (s->rc).code = uVar12 - uVar4;
              *puVar19 = uVar1 - (uVar1 >> 5);
              uVar15 = uVar15 + 1;
            }
            if (0xff < uVar15) break;
            uVar9 = (s->rc).range;
            uVar12 = (s->rc).code;
            uVar5 = uVar7;
          }
        }
        (s->dict).pos = uVar2 + 1;
        puVar17[uVar2] = (uint8_t)uVar15;
        uVar2 = (s->dict).pos;
        if ((s->dict).full < uVar2) {
          (s->dict).full = uVar2;
        }
        lVar13 = (s->lzma).state;
        if (lVar13 < STATE_MATCH_LIT) {
          (s->lzma).state = STATE_LIT_LIT;
        }
        else if (lVar13 < STATE_NONLIT_MATCH) {
          (s->lzma).state = lVar13 + ~STATE_REP_LIT_LIT;
        }
        else {
          (s->lzma).state = lVar13 + ~STATE_REP_LIT;
        }
      }
      else {
        uVar12 = uVar12 - uVar9;
        (s->rc).code = uVar12;
        (s->lzma).is_match[uVar5][uVar7] = uVar1 - (uVar1 >> 5);
        uVar15 = uVar15 - uVar9;
        if (uVar15 < 0x1000000) {
          sVar6 = (s->rc).in_pos;
          uVar15 = uVar15 * 0x100;
          (s->rc).range = uVar15;
          (s->rc).in_pos = sVar6 + 1;
          uVar12 = uVar12 * 0x100 + (uint)(s->rc).in[sVar6];
          (s->rc).code = uVar12;
        }
        uVar1 = (s->lzma).is_rep[uVar5];
        uVar2 = (uVar15 >> 0xb) * (uint)uVar1;
        if (uVar12 < uVar2) {
          (s->rc).range = uVar2;
          (s->lzma).is_rep[uVar5] = uVar1 + (short)((int)(0x800 - (uint)uVar1) >> 5);
          lVar13 = STATE_LIT_MATCH;
          if (6 < uVar5) {
            lVar13 = STATE_NONLIT_MATCH;
          }
          (s->lzma).state = lVar13;
          (s->lzma).rep3 = (s->lzma).rep2;
          (s->lzma).rep2 = (s->lzma).rep1;
          (s->lzma).rep1 = (s->lzma).rep0;
          lzma_len(s,&(s->lzma).match_len_dec,uVar7);
          uVar2 = (s->lzma).len;
          if (5 < uVar2) {
            uVar2 = 5;
          }
          uVar9 = 1;
          do {
            while( true ) {
              uVar5 = (s->rc).range;
              uVar15 = uVar9 * 2;
              uVar7 = (s->rc).code;
              puVar19 = (s->lzma).is_rep0_long[uVar2 * 4 + 4] + uVar9;
              if (uVar5 < 0x1000000) {
                sVar6 = (s->rc).in_pos;
                uVar5 = uVar5 << 8;
                (s->rc).range = uVar5;
                (s->rc).in_pos = sVar6 + 1;
                uVar7 = uVar7 * 0x100 + (uint)(s->rc).in[sVar6];
                (s->rc).code = uVar7;
              }
              uVar1 = *puVar19;
              uVar9 = (uVar5 >> 0xb) * (uint)uVar1;
              if (uVar9 <= uVar7) break;
              (s->rc).range = uVar9;
              *puVar19 = (short)((int)(0x800 - (uint)uVar1) >> 5) + uVar1;
              uVar9 = uVar15;
              if (0x3f < uVar15) goto LAB_220145ce;
            }
            (s->rc).range = uVar5 - uVar9;
            (s->rc).code = uVar7 - uVar9;
            *puVar19 = uVar1 - (uVar1 >> 5);
            uVar9 = uVar15 + 1;
          } while (uVar9 < 0x40);
LAB_220145ce:
          uVar8 = uVar9 - 0x40;
          if (uVar8 < 4) {
            (s->lzma).rep0 = uVar8;
          }
          else {
            uVar14 = (uVar8 & 1) + 2;
            (s->lzma).rep0 = uVar14;
            uVar9 = (s->rc).range;
            uVar2 = (s->rc).code;
            if (uVar8 < 0xe) {
              uVar15 = (uVar8 >> 1) - 1;
              uVar14 = uVar14 << (uVar15 & 0x1f);
              iVar10 = 0x7fffffff - uVar8;
              (s->lzma).rep0 = uVar14;
              uVar5 = 0;
              iVar11 = 1;
              uVar8 = uVar14;
              while( true ) {
                puVar16 = (ushort *)((int)s + (iVar10 + uVar14 + iVar11) * 2 + 0x5d4);
                if (uVar9 < 0x1000000) {
                  sVar6 = (s->rc).in_pos;
                  uVar9 = uVar9 << 8;
                  (s->rc).range = uVar9;
                  (s->rc).in_pos = sVar6 + 1;
                  uVar2 = (uint)(s->rc).in[sVar6] + uVar2 * 0x100;
                  (s->rc).code = uVar2;
                }
                uVar1 = *puVar16;
                iVar11 = iVar11 * 2;
                uVar7 = (uVar9 >> 0xb) * (uint)uVar1;
                if (uVar2 < uVar7) {
                  (s->rc).range = uVar7;
                  *puVar16 = uVar1 + (short)((int)(0x800 - (uint)uVar1) >> 5);
                }
                else {
                  (s->rc).range = uVar9 - uVar7;
                  (s->rc).code = uVar2 - uVar7;
                  *puVar16 = uVar1 - (uVar1 >> 5);
                  uVar8 = uVar8 + (1 << (uVar5 & 0x1f));
                  (s->lzma).rep0 = uVar8;
                  iVar11 = iVar11 + 1;
                }
                uVar5 = uVar5 + 1;
                if (uVar15 == uVar5) break;
                uVar9 = (s->rc).range;
                uVar2 = (s->rc).code;
              }
            }
            else {
              uVar15 = (uVar8 >> 1) - 5;
              do {
                uVar15 = uVar15 - 1;
                if (uVar9 < 0x1000000) {
                  sVar6 = (s->rc).in_pos;
                  uVar9 = uVar9 << 8;
                  (s->rc).range = uVar9;
                  (s->rc).in_pos = sVar6 + 1;
                  uVar2 = (uint)(s->rc).in[sVar6] + uVar2 * 0x100;
                }
                uVar9 = uVar9 >> 1;
                uVar5 = (int)(uVar2 - uVar9) >> 0x1f;
                uVar2 = (uVar2 - uVar9) + (uVar9 & uVar5);
                uVar14 = uVar14 * 2 + 1 + uVar5;
                (s->rc).range = uVar9;
                (s->rc).code = uVar2;
                (s->lzma).rep0 = uVar14;
              } while (uVar15 != 0);
              uVar8 = uVar14 * 0x10;
              (s->lzma).rep0 = uVar8;
              iVar10 = 1;
              while( true ) {
                iVar11 = iVar10 * 2;
                puVar19 = (s->lzma).dist_align + iVar10;
                if (uVar9 < 0x1000000) {
                  sVar6 = (s->rc).in_pos;
                  uVar9 = uVar9 << 8;
                  (s->rc).range = uVar9;
                  (s->rc).in_pos = sVar6 + 1;
                  uVar2 = (uint)(s->rc).in[sVar6] + uVar2 * 0x100;
                  (s->rc).code = uVar2;
                }
                uVar1 = *puVar19;
                uVar5 = (uVar9 >> 0xb) * (uint)uVar1;
                if (uVar2 < uVar5) {
                  (s->rc).range = uVar5;
                  *puVar19 = uVar1 + (short)((int)(0x800 - (uint)uVar1) >> 5);
                }
                else {
                  (s->rc).range = uVar9 - uVar5;
                  (s->rc).code = uVar2 - uVar5;
                  *puVar19 = uVar1 - (uVar1 >> 5);
                  uVar8 = uVar8 + (1 << (uVar15 & 0x1f));
                  (s->lzma).rep0 = uVar8;
                  iVar11 = iVar11 + 1;
                }
                uVar15 = uVar15 + 1;
                if (uVar15 == 4) break;
                uVar9 = (s->rc).range;
                uVar2 = (s->rc).code;
                iVar10 = iVar11;
              }
            }
          }
        }
        else {
          uVar12 = uVar12 - uVar2;
          (s->rc).code = uVar12;
          (s->lzma).is_rep[uVar5] = uVar1 - (uVar1 >> 5);
          uVar15 = uVar15 - uVar2;
          if (uVar15 < 0x1000000) {
            sVar6 = (s->rc).in_pos;
            uVar15 = uVar15 * 0x100;
            (s->rc).range = uVar15;
            (s->rc).in_pos = sVar6 + 1;
            uVar12 = uVar12 * 0x100 + (uint)(s->rc).in[sVar6];
            (s->rc).code = uVar12;
          }
          uVar1 = (s->lzma).is_rep0[uVar5];
          uVar2 = (uVar15 >> 0xb) * (uint)uVar1;
          if (uVar12 < uVar2) {
            (s->lzma).is_rep0[uVar5] = uVar1 + (short)((int)(0x800 - (uint)uVar1) >> 5);
            if (uVar2 < 0x1000000) {
              sVar6 = (s->rc).in_pos;
              uVar2 = uVar2 * 0x100;
              (s->rc).range = uVar2;
              (s->rc).in_pos = sVar6 + 1;
              uVar12 = uVar12 * 0x100 + (uint)(s->rc).in[sVar6];
              (s->rc).code = uVar12;
            }
            uVar1 = (s->lzma).is_rep0_long[uVar5][uVar7];
            uVar9 = (uVar2 >> 0xb) * (uint)uVar1;
            if (uVar12 < uVar9) {
              (s->rc).range = uVar9;
              (s->lzma).is_rep0_long[uVar5][uVar7] =
                   uVar1 + (short)((int)(0x800 - (uint)uVar1) >> 5);
              if (uVar5 < 7) {
                (s->lzma).state = STATE_LIT_SHORTREP;
                uVar8 = (s->lzma).rep0;
                (s->lzma).len = 1;
              }
              else {
                (s->lzma).state = STATE_NONLIT_REP;
                uVar8 = (s->lzma).rep0;
                (s->lzma).len = 1;
              }
              goto LAB_220145da;
            }
            (s->rc).range = uVar2 - uVar9;
            (s->rc).code = uVar12 - uVar9;
            (s->lzma).is_rep0_long[uVar5][uVar7] = uVar1 - (uVar1 >> 5);
          }
          else {
            uVar12 = uVar12 - uVar2;
            (s->rc).code = uVar12;
            (s->lzma).is_rep0[uVar5] = uVar1 - (uVar1 >> 5);
            uVar15 = uVar15 - uVar2;
            if (uVar15 < 0x1000000) {
              sVar6 = (s->rc).in_pos;
              uVar15 = uVar15 * 0x100;
              (s->rc).range = uVar15;
              (s->rc).in_pos = sVar6 + 1;
              uVar12 = uVar12 * 0x100 + (uint)(s->rc).in[sVar6];
              (s->rc).code = uVar12;
            }
            uVar1 = (s->lzma).is_rep1[uVar5];
            uVar8 = (s->lzma).rep1;
            uVar2 = (uVar15 >> 0xb) * (uint)uVar1;
            if (uVar12 < uVar2) {
              (s->rc).range = uVar2;
              (s->lzma).is_rep1[uVar5] = uVar1 + (short)((int)(0x800 - (uint)uVar1) >> 5);
              uVar18 = uVar8;
            }
            else {
              uVar12 = uVar12 - uVar2;
              (s->rc).code = uVar12;
              uVar15 = uVar15 - uVar2;
              (s->lzma).is_rep1[uVar5] = uVar1 - (uVar1 >> 5);
              if (uVar15 < 0x1000000) {
                sVar6 = (s->rc).in_pos;
                uVar15 = uVar15 * 0x100;
                (s->rc).range = uVar15;
                (s->rc).in_pos = sVar6 + 1;
                uVar12 = uVar12 * 0x100 + (uint)(s->rc).in[sVar6];
                (s->rc).code = uVar12;
              }
              uVar1 = (s->lzma).is_rep2[uVar5];
              uVar14 = (s->lzma).rep2;
              uVar2 = (uVar15 >> 0xb) * (uint)uVar1;
              if (uVar12 < uVar2) {
                (s->rc).range = uVar2;
                (s->lzma).is_rep2[uVar5] = uVar1 + (short)((int)(0x800 - (uint)uVar1) >> 5);
                (s->lzma).rep2 = uVar8;
                uVar18 = uVar14;
              }
              else {
                uVar18 = (s->lzma).rep3;
                (s->rc).range = uVar15 - uVar2;
                (s->rc).code = uVar12 - uVar2;
                (s->lzma).is_rep2[uVar5] = uVar1 - (uVar1 >> 5);
                (s->lzma).rep3 = uVar14;
                (s->lzma).rep2 = uVar8;
              }
            }
            uVar8 = (s->lzma).rep0;
            (s->lzma).rep0 = uVar18;
            (s->lzma).rep1 = uVar8;
          }
          lVar13 = STATE_LIT_LONGREP;
          if (6 < uVar5) {
            lVar13 = STATE_NONLIT_REP;
          }
          (s->lzma).state = lVar13;
          lzma_len(s,&(s->lzma).rep_len_dec,uVar7);
          uVar8 = (s->lzma).rep0;
        }
LAB_220145da:
        if ((((s->dict).full <= uVar8) || ((s->dict).size <= uVar8)) ||
           (_Var3 = dict_repeat(&s->dict,&(s->lzma).len,uVar8), CONCAT31(extraout_var,_Var3) == 0))
        {
          _Var3 = false;
          goto LAB_220144be;
        }
        uVar2 = (s->dict).pos;
      }
    } while (uVar2 < (s->dict).limit);
  }
LAB_22014490:
  uVar15 = (s->rc).range;
LAB_22014494:
  _Var3 = true;
  if (uVar15 < 0x1000000) {
    sVar6 = (s->rc).in_pos;
    (s->rc).range = uVar15 << 8;
    (s->rc).in_pos = sVar6 + 1;
    (s->rc).code = (uint)(s->rc).in[sVar6] + (s->rc).code * 0x100;
  }
LAB_220144be:
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return _Var3;
}



xz_ret xz_dec_lzma2_run(xz_dec_lzma2 *s,xz_buf *b)

{
  byte bVar1;
  xz_mode xVar2;
  xz_ret xVar3;
  _Bool _Var4;
  size_t sVar5;
  uint32_t uVar6;
  undefined3 extraout_var;
  undefined3 extraout_var_00;
  uint8_t *puVar7;
  uint uVar8;
  size_t sVar9;
  uint uVar10;
  lzma2_seq lVar11;
  uint32_t uVar12;
  uint8_t *puVar13;
  uint uVar14;
  uint16_t (*pauVar15) [16];
  size_t sVar16;
  size_t sVar17;
  uint8_t *__dest;
  
  lVar11 = (s->lzma2).sequence;
  sVar16 = b->in_size;
  __dest = (s->temp).buf;
  sVar17 = b->in_pos;
LAB_22014a9c:
  if (sVar17 < sVar16) {
    if (lVar11 == SEQ_COMPRESSED_1) {
      b->in_pos = sVar17 + 1;
      lVar11 = (s->lzma2).next_sequence;
      (s->lzma2).compressed = (uint)b->in[sVar17] + (s->lzma2).compressed + 1;
      (s->lzma2).sequence = lVar11;
      sVar17 = sVar17 + 1;
      goto LAB_22014a9c;
    }
    if (lVar11 < SEQ_PROPERTIES) {
      if (lVar11 == SEQ_UNCOMPRESSED_2) {
        b->in_pos = sVar17 + 1;
        (s->lzma2).uncompressed = (uint)b->in[sVar17] + (s->lzma2).uncompressed + 1;
        (s->lzma2).sequence = SEQ_COMPRESSED_0;
        lVar11 = SEQ_COMPRESSED_0;
        sVar17 = sVar17 + 1;
      }
      else if (lVar11 == SEQ_COMPRESSED_0) {
        b->in_pos = sVar17 + 1;
        bVar1 = b->in[sVar17];
        (s->lzma2).sequence = SEQ_COMPRESSED_1;
        (s->lzma2).compressed = (uint)bVar1 << 8;
        lVar11 = SEQ_COMPRESSED_1;
        sVar17 = sVar17 + 1;
      }
      else if (lVar11 == SEQ_CONTROL) {
        sVar5 = sVar17 + 1;
        b->in_pos = sVar5;
        uVar10 = (uint)b->in[sVar17];
        if (uVar10 == 0) {
          xVar3 = XZ_STREAM_END;
          goto LAB_22014c3a;
        }
        if ((uVar10 < 0xe0) && (uVar10 != 1)) {
          if ((s->lzma2).need_dict_reset != false) goto LAB_22014d6a;
        }
        else {
          xVar2 = (s->dict).mode;
          *(undefined2 *)&(s->lzma2).need_dict_reset = 0x100;
          if (xVar2 == XZ_SINGLE) {
            sVar17 = b->out_pos;
            sVar9 = b->out_size;
            (s->dict).buf = b->out + sVar17;
            (s->dict).end = sVar9 - sVar17;
          }
          (s->dict).start = 0;
          (s->dict).pos = 0;
          (s->dict).limit = 0;
          (s->dict).full = 0;
        }
        sVar17 = sVar5;
        if (uVar10 < 0x80) {
          if (2 < uVar10) goto LAB_22014d6a;
          *(undefined2 *)&s->lzma2 = 0x803;
          lVar11 = SEQ_COMPRESSED_0;
        }
        else {
          (s->lzma2).uncompressed = (uVar10 & 0x1f) << 0x10;
          (s->lzma2).sequence = SEQ_UNCOMPRESSED_1;
          if (uVar10 < 0xc0) {
            if ((s->lzma2).need_props != false) goto LAB_22014d6a;
            (s->lzma2).next_sequence = SEQ_LZMA_PREPARE;
            lVar11 = SEQ_UNCOMPRESSED_1;
            if (0x9f < uVar10) {
              (s->lzma).state = STATE_LIT_LIT;
              (s->lzma).rep0 = 0;
              (s->lzma).rep1 = 0;
              (s->lzma).rep2 = 0;
              (s->lzma).rep3 = 0;
              pauVar15 = (s->lzma).is_match;
              do {
                (*pauVar15)[0] = 0x400;
                pauVar15 = (uint16_t (*) [16])(*pauVar15 + 1);
              } while (&s->temp != (anon_struct_conflictbb9d_for_temp *)pauVar15);
              (s->rc).range = 0xffffffff;
              (s->rc).init_bytes_left = 5;
              (s->rc).code = 0;
              lVar11 = SEQ_UNCOMPRESSED_1;
            }
          }
          else {
            (s->lzma2).next_sequence = SEQ_PROPERTIES;
            (s->lzma2).need_props = false;
            lVar11 = SEQ_UNCOMPRESSED_1;
          }
        }
      }
      else if (lVar11 == SEQ_UNCOMPRESSED_1) {
        b->in_pos = sVar17 + 1;
        bVar1 = b->in[sVar17];
        (s->lzma2).sequence = SEQ_UNCOMPRESSED_2;
        (s->lzma2).uncompressed = (s->lzma2).uncompressed + (uint)bVar1 * 0x100;
        lVar11 = SEQ_UNCOMPRESSED_2;
        sVar17 = sVar17 + 1;
      }
      goto LAB_22014a9c;
    }
    if (lVar11 == SEQ_LZMA_PREPARE) {
      uVar10 = (s->lzma2).compressed;
      if (uVar10 < 5) goto LAB_22014d6a;
      uVar12 = (s->rc).init_bytes_left;
      if (uVar12 != 0) goto LAB_22014d2c;
      puVar7 = b->in;
    }
    else {
      if (SEQ_LZMA_PREPARE < lVar11) {
        if (lVar11 == SEQ_LZMA_RUN) goto LAB_22014aa6;
        if (lVar11 == SEQ_COPY) {
          uVar10 = (s->lzma2).compressed;
          if (uVar10 != 0) {
            uVar8 = b->out_pos;
            while( true ) {
              if (b->out_size <= uVar8) goto LAB_22014c38;
              sVar5 = (s->dict).pos;
              uVar8 = b->out_size - uVar8;
              uVar14 = (s->dict).end - sVar5;
              if (uVar8 < uVar14) {
                uVar14 = uVar8;
              }
              if (uVar10 < uVar14) {
                uVar14 = uVar10;
              }
              if (sVar16 - sVar17 < uVar14) {
                uVar14 = sVar16 - sVar17;
              }
              puVar13 = (s->dict).buf;
              puVar7 = b->in;
              (s->lzma2).compressed = uVar10 - uVar14;
              memcpy(puVar13 + sVar5,puVar7 + sVar17,uVar14);
              sVar16 = (s->dict).pos + uVar14;
              (s->dict).pos = sVar16;
              if ((s->dict).full < sVar16) {
                (s->dict).full = sVar16;
              }
              if ((s->dict).mode != XZ_SINGLE) {
                if (sVar16 == (s->dict).end) {
                  (s->dict).pos = 0;
                }
                memcpy(b->out + b->out_pos,b->in + b->in_pos,uVar14);
                sVar16 = (s->dict).pos;
              }
              sVar5 = b->out_pos;
              sVar17 = b->in_pos;
              uVar10 = (s->lzma2).compressed;
              (s->dict).start = sVar16;
              uVar8 = sVar5 + uVar14;
              sVar17 = sVar17 + uVar14;
              b->out_pos = uVar8;
              b->in_pos = sVar17;
              sVar16 = b->in_size;
              if (uVar10 == 0) break;
              if (sVar16 <= sVar17) goto LAB_22014c38;
            }
          }
          (s->lzma2).sequence = SEQ_CONTROL;
          lVar11 = SEQ_CONTROL;
        }
        goto LAB_22014a9c;
      }
      if (lVar11 != SEQ_PROPERTIES) goto LAB_22014a9c;
      b->in_pos = sVar17 + 1;
      uVar12 = (uint32_t)b->in[sVar17];
      if (0xe0 < uVar12) {
LAB_22014d6a:
        xVar3 = XZ_DATA_ERROR;
LAB_22014c3a:
        gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
        return xVar3;
      }
      if (uVar12 < 0x2d) {
        uVar6 = 0;
      }
      else {
        uVar10 = 0;
        do {
          uVar12 = uVar12 - 0x2d & 0xff;
          uVar10 = uVar10 + 1;
        } while (0x2c < uVar12);
        uVar6 = (1 << (uVar10 & 0x1f)) - 1;
      }
      (s->lzma).pos_mask = uVar6;
      (s->lzma).literal_pos_mask = 0;
      if (uVar12 < 9) {
        uVar10 = 0;
      }
      else {
        uVar10 = 0;
        do {
          uVar12 = uVar12 - 9 & 0xff;
          uVar10 = uVar10 + 1;
        } while (8 < uVar12);
        (s->lzma).literal_pos_mask = uVar10;
      }
      (s->lzma).lc = uVar12;
      if (4 < uVar12 + uVar10) goto LAB_22014d6a;
      (s->lzma).literal_pos_mask = (1 << (uVar10 & 0x1f)) - 1;
      (s->lzma).state = STATE_LIT_LIT;
      (s->lzma).rep0 = 0;
      (s->lzma).rep1 = 0;
      (s->lzma).rep2 = 0;
      (s->lzma).rep3 = 0;
      pauVar15 = (s->lzma).is_match;
      do {
        (*pauVar15)[0] = 0x400;
        pauVar15 = (uint16_t (*) [16])(*pauVar15 + 1);
      } while (&s->temp != (anon_struct_conflictbb9d_for_temp *)pauVar15);
      (s->rc).range = 0xffffffff;
      uVar10 = (s->lzma2).compressed;
      (s->rc).init_bytes_left = 5;
      (s->lzma2).sequence = SEQ_LZMA_PREPARE;
      (s->rc).code = 0;
      if (uVar10 < 5) goto LAB_22014d6a;
      uVar12 = 5;
      sVar17 = sVar17 + 1;
LAB_22014d2c:
      do {
        sVar5 = sVar17 + 1;
        uVar12 = uVar12 - 1;
        if (sVar17 == sVar16) goto LAB_22014c38;
        puVar7 = b->in;
        uVar6 = (s->rc).code;
        b->in_pos = sVar5;
        bVar1 = puVar7[sVar17];
        (s->rc).init_bytes_left = uVar12;
        (s->rc).code = (uint)bVar1 + uVar6 * 0x100;
        sVar17 = sVar5;
      } while (uVar12 != 0);
    }
    uVar12 = uVar10 - 5;
    (s->lzma2).compressed = uVar12;
    (s->lzma2).sequence = SEQ_LZMA_RUN;
LAB_22014aaa:
    uVar8 = (s->lzma2).uncompressed;
    uVar10 = b->out_size - b->out_pos;
    if (uVar8 < uVar10) {
      uVar10 = uVar8;
    }
    sVar9 = (s->dict).end;
    sVar5 = (s->dict).pos;
    if (uVar10 < sVar9 - sVar5) {
      sVar9 = uVar10 + sVar5;
    }
    uVar6 = (s->temp).size;
    (s->dict).limit = sVar9;
    uVar10 = sVar16 - sVar17;
    if ((uVar6 == 0) && (uVar12 != 0)) {
joined_r0x22014de4:
      if (0x14 < uVar10) {
        (s->rc).in = puVar7;
        (s->rc).in_pos = sVar17;
        sVar16 = sVar16 - 0x15;
        if (uVar12 + 0x15 <= uVar10) {
          sVar16 = uVar12 + sVar17;
        }
        (s->rc).in_limit = sVar16;
        _Var4 = lzma_main(s);
        if (CONCAT31(extraout_var_00,_Var4) == 0) goto LAB_22014d6a;
        sVar17 = (s->rc).in_pos;
        uVar10 = (s->lzma2).compressed;
        if (uVar10 < sVar17 - b->in_pos) goto LAB_22014d6a;
        sVar16 = b->in_size;
        uVar12 = (b->in_pos - sVar17) + uVar10;
        (s->lzma2).compressed = uVar12;
        b->in_pos = sVar17;
        uVar10 = sVar16 - sVar17;
        if (0x14 < uVar10) goto LAB_22014afa;
        puVar7 = b->in;
      }
      if (uVar12 < uVar10) {
        uVar10 = uVar12;
      }
      memcpy(__dest,puVar7 + sVar17,uVar10);
      sVar16 = b->in_pos;
      (s->temp).size = uVar10;
      b->in_pos = sVar16 + uVar10;
    }
    else {
      uVar8 = uVar12 - uVar6;
      if (uVar10 < uVar12 - uVar6) {
        uVar8 = uVar10;
      }
      if (0x2a - uVar6 < uVar8) {
        uVar8 = 0x2a - uVar6;
      }
      memcpy(__dest + uVar6,puVar7 + sVar17,uVar8);
      uVar10 = (s->temp).size + uVar8;
      if (uVar10 == (s->lzma2).compressed) {
        memset(__dest + uVar10,0,0x3f - uVar10);
        (s->rc).in_limit = (s->temp).size + uVar8;
LAB_22014da4:
        (s->rc).in = __dest;
        (s->rc).in_pos = 0;
        _Var4 = lzma_main(s);
        if (CONCAT31(extraout_var,_Var4) == 0) goto LAB_22014d6a;
        uVar10 = (s->temp).size;
        uVar14 = (s->rc).in_pos;
        if (uVar8 + uVar10 < uVar14) goto LAB_22014d6a;
        uVar12 = (s->lzma2).compressed - uVar14;
        (s->lzma2).compressed = uVar12;
        if (uVar10 <= uVar14) {
          sVar16 = b->in_size;
          puVar7 = b->in;
          sVar17 = uVar14 + (b->in_pos - uVar10);
          b->in_pos = sVar17;
          (s->temp).size = 0;
          uVar10 = sVar16 - sVar17;
          goto joined_r0x22014de4;
        }
        (s->temp).size = uVar10 - uVar14;
        memmove(__dest,__dest + uVar14,uVar10 - uVar14);
      }
      else {
        if (0x14 < uVar10) {
          (s->rc).in_limit = uVar10 - 0x15;
          goto LAB_22014da4;
        }
        sVar16 = b->in_pos;
        (s->temp).size = uVar10;
        b->in_pos = sVar16 + uVar8;
      }
    }
LAB_22014afa:
    sVar5 = (s->dict).pos;
    sVar17 = (s->dict).start;
    sVar16 = sVar5;
    if ((s->dict).mode != XZ_SINGLE) {
      if (sVar5 == (s->dict).end) {
        (s->dict).pos = 0;
      }
      memcpy(b->out + b->out_pos,(s->dict).buf + sVar17,sVar5 - sVar17);
      sVar16 = (s->dict).pos;
    }
    sVar9 = b->out_pos;
    uVar12 = (s->lzma2).uncompressed;
    (s->dict).start = sVar16;
    sVar9 = sVar9 + (sVar5 - sVar17);
    b->out_pos = sVar9;
    uVar12 = (sVar17 + uVar12) - sVar5;
    (s->lzma2).uncompressed = uVar12;
    if (uVar12 == 0) {
      if ((((s->lzma2).compressed != 0) || ((s->lzma).len != 0)) || ((s->rc).code != 0))
      goto LAB_22014d6a;
      (s->lzma2).sequence = SEQ_CONTROL;
      sVar17 = b->in_pos;
      sVar16 = b->in_size;
      (s->rc).range = 0xffffffff;
      (s->rc).init_bytes_left = 5;
      lVar11 = SEQ_CONTROL;
      goto LAB_22014a9c;
    }
    if (b->out_size != sVar9) {
      sVar17 = b->in_pos;
      sVar16 = b->in_size;
      if (sVar17 == sVar16) {
        if ((s->temp).size < (s->lzma2).compressed) goto LAB_22014c38;
        lVar11 = (s->lzma2).sequence;
      }
      else {
        lVar11 = (s->lzma2).sequence;
      }
      goto LAB_22014a9c;
    }
  }
  else if (lVar11 == SEQ_LZMA_RUN) {
LAB_22014aa6:
    puVar7 = b->in;
    uVar12 = (s->lzma2).compressed;
    goto LAB_22014aaa;
  }
LAB_22014c38:
  xVar3 = XZ_OK;
  goto LAB_22014c3a;
}



xz_dec_lzma2 * xz_dec_lzma2_create(xz_mode mode,uint32_t dict_max)

{
  undefined3 in_register_00002029;
  xz_dec_lzma2 *p;
  uint8_t *puVar1;
  
  p = (xz_dec_lzma2 *)simple_malloc(0x6f24);
  if (p != (xz_dec_lzma2 *)0x0) {
    (p->dict).mode = mode;
    (p->dict).size_max = dict_max;
    if (CONCAT31(in_register_00002029,mode) == 1) {
      puVar1 = (uint8_t *)simple_malloc(dict_max);
      (p->dict).buf = puVar1;
      if (puVar1 == (uint8_t *)0x0) {
        simple_free(p);
        p = (xz_dec_lzma2 *)0x0;
      }
    }
    else if (CONCAT31(in_register_00002029,mode) == 2) {
      (p->dict).buf = (uint8_t *)0x0;
      (p->dict).allocated = 0;
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return p;
    }
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return p;
}



xz_ret xz_dec_lzma2_reset(xz_dec_lzma2 *s,uint8_t props)

{
  xz_mode xVar1;
  uint8_t *puVar2;
  undefined3 in_register_0000202d;
  uint uVar3;
  
  if (0x27 < CONCAT31(in_register_0000202d,props)) {
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return XZ_OPTIONS_ERROR;
  }
  xVar1 = (s->dict).mode;
  uVar3 = (props & 1) + 2 << ((CONCAT31(in_register_0000202d,props) >> 1) + 0xb & 0x1f);
  (s->dict).size = uVar3;
  if (xVar1 != XZ_SINGLE) {
    if ((s->dict).size_max < uVar3) {
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return XZ_MEMLIMIT_ERROR;
    }
    (s->dict).end = uVar3;
    if ((xVar1 == XZ_DYNALLOC) && ((s->dict).allocated < uVar3)) {
      simple_free((s->dict).buf);
      puVar2 = (uint8_t *)simple_malloc((s->dict).size);
      (s->dict).buf = puVar2;
      if (puVar2 == (uint8_t *)0x0) {
        (s->dict).allocated = 0;
        gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
        return XZ_MEM_ERROR;
      }
    }
  }
  (s->lzma).len = 0;
  (s->lzma2).sequence = SEQ_CONTROL;
  (s->lzma2).need_dict_reset = true;
  (s->temp).size = 0;
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return XZ_OK;
}



void xz_dec_lzma2_end(xz_dec_lzma2 *s)

{
  if ((s->dict).mode == XZ_SINGLE) {
    simple_free(s);
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return;
  }
  simple_free((s->dict).buf);
  simple_free(s);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



xz_ret dec_vli(xz_dec *s,uint8_t *in,size_t *in_pos,size_t in_size)

{
  byte bVar1;
  uint uVar2;
  uint uVar3;
  uint uVar4;
  uint uVar5;
  uint uVar6;
  
  if (s->pos == 0) {
    *(undefined4 *)&s->vli = 0;
    *(undefined4 *)((int)&s->vli + 4) = 0;
  }
  uVar4 = *in_pos;
  while( true ) {
    if (in_size <= uVar4) {
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return XZ_OK;
    }
    uVar6 = s->pos;
    bVar1 = in[uVar4];
    uVar4 = uVar4 + 1;
    *in_pos = uVar4;
    uVar3 = bVar1 & 0x7f;
    if ((int)(uVar6 - 0x20) < 0) {
      uVar5 = (uVar3 >> 1) >> (0x1f - uVar6 & 0x1f);
      uVar3 = uVar3 << (uVar6 & 0x1f);
    }
    else {
      uVar5 = uVar3 << (uVar6 - 0x20 & 0x1f);
      uVar3 = 0;
    }
    uVar2 = *(uint *)((int)&s->vli + 4);
    *(uint *)&s->vli = *(uint *)&s->vli | uVar3;
    *(uint *)((int)&s->vli + 4) = uVar2 | uVar5;
    if (-1 < (char)bVar1) break;
    s->pos = uVar6 + 7;
    if (uVar6 + 7 == 0x3f) {
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return XZ_DATA_ERROR;
    }
  }
  if ((bVar1 == 0) && (uVar6 != 0)) {
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return XZ_DATA_ERROR;
  }
  s->pos = 0;
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return XZ_STREAM_END;
}



_Bool fill_temp(xz_dec *s,xz_buf *b)

{
  uint n;
  size_t sVar1;
  size_t sVar2;
  uint uVar3;
  
  sVar1 = (s->temp).pos;
  n = (s->temp).size - sVar1;
  uVar3 = b->in_size - b->in_pos;
  if (uVar3 < n) {
    n = uVar3;
  }
  memcpy((s->temp).buf + sVar1,b->in + b->in_pos,n);
  sVar1 = (s->temp).pos;
  sVar2 = (s->temp).size;
  b->in_pos = b->in_pos + n;
  sVar1 = n + sVar1;
  (s->temp).pos = sVar1;
  if (sVar1 != sVar2) {
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return false;
  }
  (s->temp).pos = 0;
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return true;
}



// WARNING: Type propagation algorithm not settling

xz_ret xz_dec_run(xz_dec *s,xz_buf *b)

{
  xz_check xVar1;
  byte bVar2;
  xz_dec_tmp_sequence2 xVar3;
  bool bVar4;
  _Bool _Var5;
  undefined3 extraout_var;
  uint8_t *buf;
  undefined3 extraout_var_00;
  undefined3 extraout_var_01;
  undefined3 extraout_var_02;
  undefined3 extraout_var_03;
  undefined3 extraout_var_04;
  undefined3 extraout_var_05;
  undefined3 extraout_var_06;
  size_t sVar6;
  int iVar7;
  uint32_t uVar8;
  uint uVar9;
  int iVar10;
  uint uVar11;
  uint8_t *puVar12;
  uint uVar13;
  uint uVar14;
  int iVar15;
  uint uVar16;
  undefined4 uVar17;
  size_t sVar18;
  uint uVar19;
  xz_mode xVar20;
  xz_ret xVar21;
  int iVar22;
  size_t sVar23;
  size_t sVar24;
  
  xVar20 = s->mode;
  if (xVar20 == XZ_SINGLE) {
    s->sequence = SEQ_STREAM_HEADER;
    s->allow_buf_error = false;
    s->pos = 0;
    s->crc = 0;
    memset(&s->block,0,0x30);
    memset(&s->index,0,0x30);
    (s->temp).pos = 0;
    (s->temp).size = 0xc;
  }
  else {
    xVar20 = s->sequence;
  }
  sVar23 = b->in_pos;
  sVar24 = b->out_pos;
  s->in_start = sVar23;
LAB_22015232:
  do {
    while (xVar20 == 5) {
LAB_2201557e:
      if (s->check_type == XZ_CHECK_CRC32) {
        sVar6 = b->in_size;
        sVar18 = b->in_pos;
        do {
          if (sVar6 == sVar18) {
            if (s->mode != XZ_SINGLE) goto LAB_220153b4;
            sVar18 = b->in_pos;
            goto LAB_220152bc;
          }
          uVar11 = s->pos;
          uVar16 = s->crc;
          b->in_pos = sVar18 + 1;
          uVar9 = uVar11 + 8;
          if ((uVar16 >> (uVar11 & 0x1f) & 0xff) != (uint)b->in[sVar18]) goto LAB_2201568a;
          s->pos = uVar9;
          sVar18 = sVar18 + 1;
        } while (uVar9 < 0x20);
        s->crc = 0;
        s->pos = 0;
      }
      s->sequence = SEQ_BLOCK_START;
      xVar20 = XZ_PREALLOC;
    }
    if (xVar20 < 6) {
      if (xVar20 == XZ_DYNALLOC) {
LAB_220153da:
        _Var5 = fill_temp(s,b);
        if (CONCAT31(extraout_var_00,_Var5) == 0) goto LAB_220152ae;
        puVar12 = (s->temp).buf;
        sVar18 = (s->temp).size - 4;
        (s->temp).size = sVar18;
        uVar8 = xz_crc32(puVar12,sVar18,0);
        sVar18 = (s->temp).size;
        if (uVar8 == *(uint32_t *)(puVar12 + sVar18)) {
          bVar2 = (s->temp).buf[1];
          (s->temp).pos = 2;
          if ((bVar2 & 0x3f) == 0) {
            if ((bVar2 & 0x40) == 0) {
              *(undefined4 *)&(s->block_header).compressed = 0xffffffff;
              *(undefined4 *)((int)&(s->block_header).compressed + 4) = 0xffffffff;
            }
            else {
              xVar21 = dec_vli(s,puVar12,&(s->temp).pos,sVar18);
              if (CONCAT31(extraout_var_05,xVar21) != 1) goto LAB_220157cc;
              uVar17 = *(undefined4 *)((int)&s->vli + 4);
              bVar2 = (s->temp).buf[1];
              sVar18 = (s->temp).size;
              *(undefined4 *)&(s->block_header).compressed = *(undefined4 *)&s->vli;
              *(undefined4 *)((int)&(s->block_header).compressed + 4) = uVar17;
            }
            if ((char)bVar2 < '\0') {
              xVar21 = dec_vli(s,puVar12,&(s->temp).pos,sVar18);
              if (CONCAT31(extraout_var_06,xVar21) != 1) goto LAB_220157cc;
              uVar17 = *(undefined4 *)((int)&s->vli + 4);
              sVar18 = (s->temp).size;
              *(undefined4 *)&(s->block_header).uncompressed = *(undefined4 *)&s->vli;
              *(undefined4 *)((int)&(s->block_header).uncompressed + 4) = uVar17;
            }
            else {
              *(undefined4 *)&(s->block_header).uncompressed = 0xffffffff;
              *(undefined4 *)((int)&(s->block_header).uncompressed + 4) = 0xffffffff;
            }
            sVar6 = (s->temp).pos;
            if (sVar18 - sVar6 < 2) {
              iVar7 = 7;
            }
            else {
              (s->temp).pos = sVar6 + 1;
              if ((s->temp).buf[sVar6] == '!') {
                (s->temp).pos = sVar6 + 2;
                if ((s->temp).buf[sVar6 + 1] != '\x01') goto LAB_2201566e;
                if (sVar6 + 2 == sVar18) goto LAB_220157cc;
                (s->temp).pos = sVar6 + 3;
                xVar21 = xz_dec_lzma2_reset(s->lzma2,(s->temp).buf[sVar6 + 2]);
                iVar22 = CONCAT31(extraout_var_02,xVar21);
                if (iVar22 == 0) {
                  sVar18 = (s->temp).pos;
                  while (sVar18 < (s->temp).size) {
                    (s->temp).pos = sVar18 + 1;
                    puVar12 = (s->temp).buf + sVar18;
                    sVar18 = sVar18 + 1;
                    if (*puVar12 != '\0') goto LAB_2201566e;
                  }
                  *(undefined4 *)&(s->block).compressed = 0;
                  *(undefined4 *)&(s->block).uncompressed = 0;
                  (s->temp).pos = 0;
                  *(undefined4 *)((int)&(s->block).compressed + 4) = 0;
                  *(undefined4 *)((int)&(s->block).uncompressed + 4) = 0;
                  s->sequence = SEQ_BLOCK_UNCOMPRESS;
                  goto LAB_2201546c;
                }
                if (s->mode != XZ_SINGLE) goto LAB_220153c0;
                iVar7 = iVar22;
                if (iVar22 != 1) goto LAB_2201542c;
                goto LAB_22015962;
              }
              iVar7 = 6;
            }
            xVar21 = (xz_ret)iVar7;
            if (s->mode == XZ_SINGLE) goto LAB_2201542c;
            s->allow_buf_error = false;
          }
          else {
            iVar7 = 6;
            xVar21 = XZ_OPTIONS_ERROR;
            if (s->mode == XZ_SINGLE) goto LAB_2201542c;
            s->allow_buf_error = false;
          }
          goto LAB_22015434;
        }
      }
      else {
        if (xVar20 < (XZ_PREALLOC|XZ_DYNALLOC)) {
          if (xVar20 == XZ_SINGLE) {
            _Var5 = fill_temp(s,b);
            if (CONCAT31(extraout_var_04,_Var5) == 0) goto LAB_220152ae;
            s->sequence = SEQ_BLOCK_START;
            iVar7 = memcmp((s->temp).buf,&DAT_220179c4,6);
            if (iVar7 == 0) {
              uVar8 = xz_crc32((s->temp).buf + 6,2,0);
              if (uVar8 != *(uint32_t *)((s->temp).buf + 8)) goto LAB_220157cc;
              if (((s->temp).buf[6] == '\0') &&
                 (xVar1 = (s->temp).buf[7], s->check_type = xVar1, xVar1 < 2)) goto LAB_220152ce;
LAB_2201566e:
              xVar21 = XZ_OPTIONS_ERROR;
              iVar7 = 6;
              if (s->mode == XZ_SINGLE) goto LAB_2201542c;
              s->allow_buf_error = false;
            }
            else {
              iVar7 = 5;
              xVar21 = XZ_FORMAT_ERROR;
              if (s->mode == XZ_SINGLE) goto LAB_2201542c;
              s->allow_buf_error = false;
            }
            goto LAB_22015434;
          }
          if (xVar20 == XZ_PREALLOC) {
LAB_220152ce:
            sVar18 = b->in_pos;
            if (sVar18 == b->in_size) {
LAB_2201528c:
              if (s->mode != XZ_SINGLE) goto LAB_220153b4;
              goto LAB_22015294;
            }
            if (b->in[sVar18] != 0) {
              uVar8 = (b->in[sVar18] + 1) * 4;
              (s->block_header).size = uVar8;
              (s->temp).size = uVar8;
              (s->temp).pos = 0;
              s->sequence = SEQ_BLOCK_HEADER;
              goto LAB_220153da;
            }
            b->in_pos = sVar18 + 1;
            s->in_start = sVar18;
            s->sequence = SEQ_INDEX;
            xVar20 = 6;
          }
          goto LAB_22015232;
        }
        if (xVar20 != (XZ_PREALLOC|XZ_DYNALLOC)) {
LAB_22015248:
          uVar11 = *(uint *)&(s->block).compressed;
          iVar7 = *(int *)((int)&(s->block).compressed + 4);
          if ((uVar11 & 3) != 0) {
            sVar18 = b->in_size;
            sVar6 = b->in_pos;
            do {
              uVar9 = uVar11 + 1;
              iVar7 = iVar7 + (uint)(uVar9 < uVar11);
              if (sVar18 == sVar6) goto LAB_2201528c;
              b->in_pos = sVar6 + 1;
              if (b->in[sVar6] != '\0') goto LAB_22015424;
              *(uint *)&(s->block).compressed = uVar9;
              *(int *)((int)&(s->block).compressed + 4) = iVar7;
              sVar6 = sVar6 + 1;
              uVar11 = uVar9;
            } while ((uVar9 & 3) != 0);
          }
          s->sequence = SEQ_BLOCK_CHECK;
          goto LAB_2201557e;
        }
LAB_2201546c:
        sVar18 = b->out_pos;
        s->in_start = b->in_pos;
        s->out_start = sVar18;
        xVar21 = xz_dec_lzma2_run(s->lzma2,b);
        iVar22 = CONCAT31(extraout_var_01,xVar21);
        sVar18 = s->out_start;
        uVar13 = b->in_pos - s->in_start;
        uVar11 = b->out_pos - sVar18;
        uVar16 = *(int *)&(s->block).compressed + uVar13;
        uVar9 = *(int *)&(s->block).uncompressed + uVar11;
        uVar19 = *(uint *)((int)&(s->block_header).compressed + 4);
        uVar14 = (uint)(uVar16 < uVar13) + *(int *)((int)&(s->block).compressed + 4);
        uVar13 = (uint)(uVar9 < uVar11) + *(int *)((int)&(s->block).uncompressed + 4);
        *(uint *)&(s->block).compressed = uVar16;
        *(uint *)((int)&(s->block).compressed + 4) = uVar14;
        *(uint *)&(s->block).uncompressed = uVar9;
        *(uint *)((int)&(s->block).uncompressed + 4) = uVar13;
        if ((uVar14 <= uVar19) &&
           ((((uVar19 != uVar14 || (uVar16 <= *(uint *)&(s->block_header).compressed)) &&
             (uVar16 = *(uint *)((int)&(s->block_header).uncompressed + 4), uVar13 <= uVar16)) &&
            ((uVar16 != uVar13 || (uVar9 <= *(uint *)&(s->block_header).uncompressed)))))) {
          if (s->check_type == XZ_CHECK_CRC32) {
            uVar8 = xz_crc32(b->out + sVar18,uVar11,s->crc);
            s->crc = uVar8;
          }
          if (iVar22 != 1) goto LAB_2201583c;
          iVar22 = *(int *)&(s->block_header).compressed;
          iVar7 = *(int *)((int)&(s->block_header).compressed + 4);
          if (((iVar22 == -1) && (iVar7 == -1)) ||
             ((*(int *)&(s->block).compressed == iVar22 &&
              (*(int *)((int)&(s->block).compressed + 4) == iVar7)))) {
            iVar15 = *(int *)&(s->block_header).uncompressed;
            iVar22 = *(int *)((int)&(s->block_header).uncompressed + 4);
            iVar10 = *(int *)&(s->block).uncompressed;
            iVar7 = *(int *)((int)&(s->block).uncompressed + 4);
            if (((iVar15 == -1) && (iVar22 == -1)) || ((iVar10 == iVar15 && (iVar7 == iVar22)))) {
              uVar9 = *(uint *)&(s->block).hash.unpadded;
              uVar11 = *(int *)&(s->block).compressed + uVar9;
              uVar16 = (s->block_header).size + uVar11;
              xVar1 = s->check_type;
              iVar22 = (uint)(uVar16 < uVar11) +
                       (uint)(uVar11 < uVar9) +
                       *(int *)((int)&(s->block).hash.unpadded + 4) +
                       *(int *)((int)&(s->block).compressed + 4);
              *(uint *)&(s->block).hash.unpadded = uVar16;
              *(int *)((int)&(s->block).hash.unpadded + 4) = iVar22;
              if (xVar1 == XZ_CHECK_CRC32) {
                *(uint *)&(s->block).hash.unpadded = uVar16 + 4;
                *(uint *)((int)&(s->block).hash.unpadded + 4) = iVar22 + (uint)(uVar16 + 4 < uVar16)
                ;
              }
              uVar9 = *(uint *)&(s->block).hash.uncompressed;
              iVar22 = *(int *)((int)&(s->block).hash.uncompressed + 4);
              uVar8 = (s->block).hash.crc32;
              uVar11 = iVar10 + uVar9;
              *(uint *)&(s->block).hash.uncompressed = uVar11;
              *(uint *)((int)&(s->block).hash.uncompressed + 4) =
                   (uint)(uVar11 < uVar9) + iVar22 + iVar7;
              uVar8 = xz_crc32((uint8_t *)&(s->block).hash,0x18,uVar8);
              uVar9 = *(uint *)&(s->block).count;
              iVar7 = *(int *)((int)&(s->block).count + 4);
              (s->block).hash.crc32 = uVar8;
              uVar11 = uVar9 + 1;
              *(uint *)((int)&(s->block).count + 4) = (uint)(uVar11 < uVar9) + iVar7;
              *(uint *)&(s->block).count = uVar11;
              s->sequence = SEQ_BLOCK_PADDING;
              goto LAB_22015248;
            }
          }
        }
      }
LAB_22015424:
      iVar22 = 7;
      iVar7 = 7;
      if (s->mode != XZ_SINGLE) goto LAB_220153c0;
      goto LAB_2201542c;
    }
    if (xVar20 == 8) goto LAB_22015374;
    if (xVar20 < 9) {
      if (xVar20 == 6) {
        goto LAB_220156bc;
      }
      if (xVar20 == 7) goto LAB_220152fc;
      goto LAB_22015232;
    }
  } while (xVar20 != 9);
LAB_220152a4:
  _Var5 = fill_temp(s,b);
  if (CONCAT31(extraout_var,_Var5) != 0) {
    if ((((s->temp).buf[10] != 'Y') || ((s->temp).buf[0xb] != 'Z')) ||
       (uVar8 = xz_crc32((s->temp).buf + 4,6,0), uVar8 != *(uint32_t *)(s->temp).buf))
    goto LAB_2201568a;
    uVar11 = *(uint *)((int)&(s->index).size + 4);
    xVar20 = s->mode;
    if ((((*(uint *)&(s->index).size >> 2 | uVar11 << 0x1e) == *(uint *)((s->temp).buf + 4)) &&
        (uVar11 >> 2 == 0)) && (((s->temp).buf[8] == '\0' && ((s->temp).buf[9] == s->check_type))))
    {
      if (xVar20 == XZ_SINGLE) {
LAB_22015962:
        xVar21 = XZ_STREAM_END;
      }
      else {
        xVar21 = XZ_STREAM_END;
        s->allow_buf_error = false;
      }
      goto LAB_22015434;
    }
    goto joined_r0x2201568e;
  }
LAB_220152ae:
  xVar20 = s->mode;
  sVar18 = b->in_pos;
joined_r0x22015810:
  if (xVar20 != XZ_SINGLE) {
LAB_220153b4:
    iVar22 = 0;
    if ((sVar23 == sVar18) && (b->out_pos == sVar24)) {
      _Var5 = s->allow_buf_error;
      s->allow_buf_error = true;
      xVar21 = (_Var5 != false) << 3;
    }
    else {
LAB_220153c0:
      xVar21 = (xz_ret)iVar22;
      s->allow_buf_error = false;
    }
    goto LAB_22015434;
  }
  sVar6 = b->in_size;
LAB_220152bc:
  iVar7 = 8;
  if (sVar6 == sVar18) {
    iVar7 = 7;
  }
  goto LAB_2201542c;
LAB_220157cc:
  iVar7 = 7;
  xVar21 = XZ_DATA_ERROR;
  if (s->mode != XZ_SINGLE) {
    s->allow_buf_error = false;
    goto LAB_22015434;
  }
  goto LAB_2201542c;
LAB_220156bc:
  do {
    xVar21 = dec_vli(s,b->in,&b->in_pos,b->in_size);
    iVar22 = CONCAT31(extraout_var_03,xVar21);
    if (iVar22 != 1) {
      sVar6 = s->in_start;
      uVar9 = *(uint *)&(s->index).size;
      iVar7 = *(int *)((int)&(s->index).size + 4);
      sVar18 = b->in_pos - sVar6;
      puVar12 = b->in;
      uVar11 = uVar9 + sVar18;
      uVar8 = s->crc;
      *(uint *)&(s->index).size = uVar11;
      *(uint *)((int)&(s->index).size + 4) = (uint)(uVar11 < uVar9) + iVar7;
      uVar8 = xz_crc32(puVar12 + sVar6,sVar18,uVar8);
      s->crc = uVar8;
LAB_2201583c:
      if (s->mode != XZ_SINGLE) {
        if (iVar22 == 0) {
          sVar18 = b->in_pos;
          goto LAB_220153b4;
        }
        goto LAB_220153c0;
      }
      iVar7 = iVar22;
      if (iVar22 == 0) {
        sVar6 = b->in_size;
        sVar18 = b->in_pos;
        goto LAB_220152bc;
      }
      goto LAB_2201542c;
    }
    xVar3 = (s->index).sequence;
    if (xVar3 == SEQ_INDEX_UNPADDED) {
      uVar13 = *(uint *)&(s->index).hash.unpadded;
      iVar7 = *(int *)((int)&(s->index).hash.unpadded + 4);
      iVar22 = *(int *)((int)&s->vli + 4);
      uVar11 = *(int *)&s->vli + uVar13;
      uVar9 = *(uint *)((int)&(s->index).count + 4);
      uVar16 = *(uint *)&(s->index).count;
      *(uint *)&(s->index).hash.unpadded = uVar11;
      *(uint *)((int)&(s->index).hash.unpadded + 4) = iVar7 + iVar22 + (uint)(uVar11 < uVar13);
      (s->index).sequence = SEQ_INDEX_UNCOMPRESSED;
    }
    else if (xVar3 == SEQ_INDEX_UNCOMPRESSED) {
      uVar9 = *(uint *)&(s->index).hash.uncompressed;
      iVar22 = *(int *)((int)&(s->index).hash.uncompressed + 4);
      iVar7 = *(int *)((int)&s->vli + 4);
      uVar11 = *(int *)&s->vli + uVar9;
      uVar8 = (s->index).hash.crc32;
      *(uint *)&(s->index).hash.uncompressed = uVar11;
      *(uint *)((int)&(s->index).hash.uncompressed + 4) = (uint)(uVar11 < uVar9) + iVar22 + iVar7;
      uVar8 = xz_crc32((uint8_t *)&(s->index).hash,0x18,uVar8);
      uVar11 = *(uint *)&(s->index).count;
      iVar7 = *(int *)((int)&(s->index).count + 4);
      (s->index).hash.crc32 = uVar8;
      uVar16 = uVar11 - 1;
      uVar9 = (uint)(uVar16 < uVar11) + iVar7 + -1;
      *(uint *)&(s->index).count = uVar16;
      *(uint *)((int)&(s->index).count + 4) = uVar9;
      (s->index).sequence = SEQ_INDEX_UNPADDED;
    }
    else {
      if (xVar3 != SEQ_INDEX_COUNT) {
        uVar16 = *(uint *)&(s->index).count;
        uVar9 = *(uint *)((int)&(s->index).count + 4);
        goto LAB_220156b6;
      }
      uVar16 = *(uint *)&s->vli;
      uVar9 = *(uint *)((int)&s->vli + 4);
      uVar11 = *(uint *)&(s->block).count;
      *(uint *)&(s->index).count = uVar16;
      *(uint *)((int)&(s->index).count + 4) = uVar9;
      if ((uVar11 != uVar16) || (*(uint *)((int)&(s->block).count + 4) != uVar9)) goto LAB_2201568a;
      (s->index).sequence = SEQ_INDEX_UNPADDED;
    }
LAB_220156b6:
  } while ((uVar16 | uVar9) != 0);
  s->sequence = SEQ_INDEX_PADDING;
LAB_220152fc:
  sVar18 = b->in_pos;
  iVar7 = *(int *)&(s->index).size;
  iVar22 = *(int *)((int)&(s->index).size + 4);
  puVar12 = b->in + sVar18;
  buf = b->in + s->in_start;
  sVar6 = (int)puVar12 - (int)buf;
  uVar11 = sVar6 + iVar7;
  bVar4 = uVar11 < sVar6;
  while (iVar10 = (uint)bVar4 + iVar22, (uVar11 & 3) != 0) {
    if (b->in_size == sVar18) {
      uVar8 = s->crc;
      *(uint *)&(s->index).size = uVar11;
      *(int *)((int)&(s->index).size + 4) = iVar10;
      uVar8 = xz_crc32(buf,sVar6,uVar8);
      xVar20 = s->mode;
      s->crc = uVar8;
      sVar18 = b->in_pos;
      goto joined_r0x22015810;
    }
    sVar18 = sVar18 + 1;
    b->in_pos = sVar18;
    if (*puVar12 != '\0') goto LAB_220157cc;
    sVar6 = (int)(puVar12 + 1) - (int)buf;
    uVar11 = sVar6 + iVar7;
    bVar4 = uVar11 < sVar6;
    puVar12 = puVar12 + 1;
  }
  uVar8 = s->crc;
  *(uint *)&(s->index).size = uVar11;
  *(int *)((int)&(s->index).size + 4) = iVar10;
  uVar8 = xz_crc32(buf,sVar6,uVar8);
  s->crc = uVar8;
  iVar7 = memcmp(&(s->block).hash,&(s->index).hash,0x18);
  if (iVar7 == 0) {
    s->sequence = SEQ_INDEX_CRC32;
LAB_22015374:
    sVar6 = b->in_size;
    sVar18 = b->in_pos;
    do {
      if (sVar6 == sVar18) {
        if (s->mode == XZ_SINGLE) goto LAB_22015294;
        sVar18 = b->in_pos;
        goto LAB_220153b4;
      }
      uVar11 = s->pos;
      uVar16 = s->crc;
      b->in_pos = sVar18 + 1;
      uVar9 = uVar11 + 8;
      if ((uVar16 >> (uVar11 & 0x1f) & 0xff) != (uint)b->in[sVar18]) goto LAB_2201568a;
      s->pos = uVar9;
      sVar18 = sVar18 + 1;
    } while (uVar9 < 0x20);
    (s->temp).size = 0xc;
    s->crc = 0;
    s->pos = 0;
    s->sequence = SEQ_STREAM_FOOTER;
    goto LAB_220152a4;
  }
LAB_2201568a:
  xVar20 = s->mode;
joined_r0x2201568e:
  if (xVar20 != XZ_SINGLE) {
    xVar21 = XZ_DATA_ERROR;
    s->allow_buf_error = false;
    goto LAB_22015434;
  }
LAB_22015294:
  iVar7 = 7;
LAB_2201542c:
  xVar21 = (xz_ret)iVar7;
  b->in_pos = sVar23;
  b->out_pos = sVar24;
LAB_22015434:
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return xVar21;
}



xz_dec * xz_dec_init(xz_mode mode,uint32_t dict_max)

{
  xz_dec *p;
  xz_dec_lzma2 *pxVar1;
  
  p = (xz_dec *)simple_malloc(0x4a8);
  if (p != (xz_dec *)0x0) {
    p->mode = mode;
    pxVar1 = xz_dec_lzma2_create(mode,dict_max);
    p->lzma2 = pxVar1;
    if (pxVar1 == (xz_dec_lzma2 *)0x0) {
      simple_free(p);
      p = (xz_dec *)0x0;
    }
    else {
      p->sequence = SEQ_STREAM_HEADER;
      p->allow_buf_error = false;
      p->pos = 0;
      p->crc = 0;
      memset(&p->block,0,0x30);
      memset(&p->index,0,0x30);
      (p->temp).pos = 0;
      (p->temp).size = 0xc;
    }
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return p;
}



void xz_dec_end(xz_dec *s)

{
  if (s != (xz_dec *)0x0) {
    xz_dec_lzma2_end(s->lzma2);
    simple_free(s);
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return;
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



void simple_malloc_init(uint8_t *buf,uint32_t len)

{
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  bufsize = len;
  mallocBuf = buf;
  malloced = 0;
  return;
}



void * simple_malloc(uint32_t size)

{
  uint8_t *puVar1;
  
  bflb_platform_printf("Simple Malloc %d\r\n",size);
  puVar1 = (uint8_t *)0x0;
  if (size + malloced < bufsize) {
    puVar1 = mallocBuf + malloced;
    malloced = size + malloced;
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return puVar1;
}



void simple_free(void *p)

{
  bflb_platform_printf("Simple Free %08x\r\n",p);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SystemInit(void)

{
  HBN_BOR_CFG_Type aHStack20 [4];
  
  _DAT_4000f014 = _DAT_4000f014 & 0xfffeffff;
  _DAT_4000007c = _DAT_4000007c & 0xfffffff0;
  if ((_DAT_4000e418 & 0xffffff) == 0x49d39d) {
    _DAT_4000e418 = _DAT_4000e418 & 0xff000000 | 0x49d89e;
  }
  _DAT_40000080 = _DAT_40000080 & 0xf8ffff03;
  memset((void *)0x2800400,0,0x50);
  memset((void *)0x2800000,0,0x50);
  aHStack20[0] = (HBN_BOR_CFG_Type)0x1010001;
  HBN_Set_BOR_Cfg(aHStack20);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Removing unreachable block (ram,0x22015c0e)
// WARNING: Removing unreachable block (ram,0x22015be0)
// WARNING: Removing unreachable block (ram,0x22015bb2)
// WARNING: Removing unreachable block (ram,0x22015c7a)

void start_load(void)

{
  memcpy(&flashCfg_Gd_Q80E_Q16E,&__text_code_end__,0xec);
  memset(eflash_loader_cmd_ack_buf,0,0xdbbc);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



void Interrupt_Handler_Stub(void)

{
  Interrupt_Handler();
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



void Trap_Handler_Stub(void)

{
  Trap_Handler();
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



void clic_enable_interrupt(uint32_t source)

{
  *(undefined *)(source + 0x2800400) = 1;
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



void clic_disable_interrupt(uint32_t source)

{
  *(undefined *)(source + 0x2800400) = 0;
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



void clic_clear_pending(uint32_t source)

{
  *(undefined *)(source + 0x2800000) = 0;
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Removing unreachable block (ram,0x22015e3e)
// WARNING: Removing unreachable block (ram,0x22015e50)
// WARNING: Removing unreachable block (ram,0x22015e62)

void Trap_Handler(void)

{
  uint uVar1;
  undefined4 in_mepc;
  uint in_mcause;
  undefined4 in_mtval;
  
  bflb_platform_printf("Trap_Handler\r\n");
  bflb_platform_printf("mcause=%08x\r\n",in_mcause);
  bflb_platform_printf("mepc:%08x\r\n",in_mepc);
  bflb_platform_printf("mtval:%08x\r\n",in_mtval);
  uVar1 = in_mcause & 0x3ff;
  if (uVar1 == 5) {
    bflb_platform_printf("Load access fault\r\n");
    goto LAB_22015e9e;
  }
  if (uVar1 < 6) {
    if (uVar1 == 3) {
      bflb_platform_printf("Breakpoint\r\n");
      goto LAB_22015e9e;
    }
    if ((in_mcause & 0x3fc) == 0) {
      if (uVar1 == 1) {
        bflb_platform_printf("Instruction access fault\r\n");
        goto LAB_22015e9e;
      }
      if (uVar1 == 2) {
        bflb_platform_printf("Illegal instruction\r\n");
        goto LAB_22015e9e;
      }
    }
    else if (uVar1 == 4) {
      bflb_platform_printf("Load address misaligned\r\n");
      goto LAB_22015e9e;
    }
  }
  else {
    if (uVar1 == 8) {
      bflb_platform_printf("Environment call from U-mode\r\n");
      goto LAB_22015e9e;
    }
    if (uVar1 < 9) {
      if (uVar1 == 6) {
        bflb_platform_printf("Store/AMO address misaligned\r\n");
        goto LAB_22015e9e;
      }
      if (uVar1 == 7) {
        bflb_platform_printf("Store/AMO access fault\r\n");
        goto LAB_22015e9e;
      }
    }
    else if (uVar1 == 9) {
      bflb_platform_printf("Environment call from M-mode\r\n");
      goto LAB_22015e9e;
    }
  }
  bflb_platform_printf("Cause num=%d\r\n");
LAB_22015e9e:
  do {
                    // WARNING: Do nothing block with infinite loop
  } while( true );
}



void Interrupt_Handler_Register(IRQn_Type irq,pFunc *interruptFun)

{
  undefined3 in_register_00002029;
  
  if (CONCAT31(in_register_00002029,irq) < 0x50) {
    __Interrupt_Handlers[CONCAT31(in_register_00002029,irq)] = interruptFun;
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Removing unreachable block (ram,0x22015f7a)
// WARNING: Removing unreachable block (ram,0x22015f80)
// WARNING: Exceeded maximum restarts with more pending

void Interrupt_Handler(void)

{
  uint uVar1;
  uint in_mcause;
  uint32_t ulMEPC;
  uint32_t ulMCAUSE;
  
  if (-1 < (int)in_mcause) {
    bflb_platform_printf("Exception should not be here\r\n");
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return;
  }
  uVar1 = in_mcause & 0x3ff;
  if (uVar1 < 0x50) {
    if (__Interrupt_Handlers[uVar1] == (pFunc *)0x0) {
      bflb_platform_printf("Interrupt num:%d IRQHandler not installed\r\n");
      if ((in_mcause & 0x3f0) != 0) {
        bflb_platform_printf("Peripheral Interrupt num:%d \r\n",uVar1 - 0x10);
      }
      do {
                    // WARNING: Do nothing block with infinite loop
      } while( true );
    }
                    // WARNING: Could not emulate address calculation at 0x22015fa8
                    // WARNING: Treating indirect jump as call
    (*__Interrupt_Handlers[uVar1])();
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return;
  }
  bflb_platform_printf("Unexpected interrupt num:%d\r\n");
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Exceeded maximum restarts with more pending

void UART_IntHandler(UART_ID_Type uartId)

{
  uint uVar1;
  undefined3 in_register_00002029;
  int iVar2;
  intCallback_Type *piVar3;
  uint uVar4;
  uint32_t uVar5;
  
  iVar2 = CONCAT31(in_register_00002029,uartId);
  uVar5 = uartAddr[iVar2];
  uVar1 = *(uint *)(uVar5 + 0x20);
  uVar4 = *(uint *)(uVar5 + 0x24);
  if (((uVar1 & 1) != 0) && ((uVar4 & 1) == 0)) {
    piVar3 = uartIntCbfArra[iVar2][0];
    *(undefined4 *)(uVar5 + 0x28) = 1;
    if (piVar3 != (intCallback_Type *)0x0) {
      (*piVar3)();
    }
  }
  if (((uVar1 & 2) != 0) && ((uVar4 & 2) == 0)) {
    piVar3 = uartIntCbfArra[iVar2][1];
    *(undefined4 *)(uVar5 + 0x28) = 2;
    if (piVar3 != (intCallback_Type *)0x0) {
      (*piVar3)();
    }
  }
  if ((((uVar1 & 4) != 0) && ((uVar4 & 4) == 0)) &&
     (uartIntCbfArra[iVar2][2] != (intCallback_Type *)0x0)) {
    (*uartIntCbfArra[iVar2][2])();
  }
  if ((((uVar1 & 8) != 0) && ((uVar4 & 8) == 0)) &&
     (uartIntCbfArra[iVar2][3] != (intCallback_Type *)0x0)) {
    (*uartIntCbfArra[iVar2][3])();
  }
  if (((uVar1 & 0x10) != 0) && ((uVar4 & 0x10) == 0)) {
    piVar3 = uartIntCbfArra[iVar2][4];
    *(undefined4 *)(uVar5 + 0x28) = 0x10;
    if (piVar3 != (intCallback_Type *)0x0) {
      (*piVar3)();
    }
  }
  if (((uVar1 & 0x20) != 0) && ((uVar4 & 0x20) == 0)) {
    piVar3 = uartIntCbfArra[iVar2][5];
    *(undefined4 *)(uVar5 + 0x28) = 0x20;
    if (piVar3 != (intCallback_Type *)0x0) {
      (*piVar3)();
    }
  }
  if ((((uVar1 & 0x40) != 0) && ((uVar4 & 0x40) == 0)) &&
     (uartIntCbfArra[iVar2][6] != (intCallback_Type *)0x0)) {
    (*uartIntCbfArra[iVar2][6])();
  }
  if ((((uVar1 & 0x80) != 0) && ((uVar4 & 0x80) == 0)) &&
     (uartIntCbfArra[iVar2][7] != (intCallback_Type *)0x0)) {
                    // WARNING: Could not recover jumptable at 0x220160a8. Too many branches
                    // WARNING: Treating indirect jump as call
    (*uartIntCbfArra[iVar2][7])();
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return;
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



void UART0_IRQHandler(void)

{
  UART_IntHandler(UART0_ID);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



void UART1_IRQHandler(void)

{
  UART_IntHandler(UART1_ID);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



BL_Err_Type UART_Init(UART_ID_Type uartId,UART_CFG_Type *uartCfg)

{
  UART_Parity_Type UVar1;
  undefined3 in_register_00002029;
  uint *puVar2;
  uint uVar3;
  uint uVar4;
  uint uVar5;
  uint uVar6;
  
  puVar2 = (uint *)uartAddr[CONCAT31(in_register_00002029,uartId)];
  uVar3 = uartCfg->uartClk / uartCfg->baudRate -
          (uint)(((uartCfg->uartClk * 10) / uartCfg->baudRate) % 10 < 5);
  puVar2[2] = uVar3 & 0xffff | uVar3 * 0x10000;
  UVar1 = uartCfg->parity;
  uVar4 = *puVar2;
  uVar3 = puVar2[1];
  if (UVar1 == UART_PARITY_ODD) {
    uVar4 = uVar4 | 0x30;
    uVar3 = uVar3 | 0x30;
  }
  else if (UVar1 == UART_PARITY_EVEN) {
    uVar4 = uVar4 & 0xffffffdf | 0x10;
    uVar3 = uVar3 & 0xffffffdf | 0x10;
  }
  else if (UVar1 == UART_PARITY_NONE) {
    uVar4 = uVar4 & 0xffffffef;
    uVar3 = uVar3 & 0xffffffef;
  }
  uVar6 = (uartCfg->dataBits + 4) * 0x100;
  uVar5 = uVar4 & 0xfffff8ff | uVar6;
  uVar4 = (uartCfg->stopBits + 1) * 0x1000;
  uVar6 = uVar6 | uVar3 & 0xfffff8ff;
  uVar3 = uVar5 & 0xffffcffd | uVar4;
  if (uartCfg->ctsFlowControl == ENABLE) {
    uVar3 = uVar5 & 0xffffcfff | uVar4 | 2;
  }
  if (uartCfg->rxDeglitch == ENABLE) {
    uVar6 = uVar6 | 0x800;
  }
  else {
    uVar6 = uVar6 & 0xfffff7ff;
  }
  uVar4 = uVar6 & 0xfffffffd;
  if (uartCfg->rtsSoftwareControl == ENABLE) {
    uVar4 = uVar6 | 2;
  }
  *puVar2 = uVar3;
  puVar2[1] = uVar4;
  uVar3 = puVar2[3] & 0xfffffffe;
  if (uartCfg->byteBitInverse == UART_MSB_FIRST) {
    uVar3 = puVar2[3] | 1;
  }
  puVar2[3] = uVar3;
  Interrupt_Handler_Register(UART0_IRQn,UART0_IRQHandler);
  Interrupt_Handler_Register(UART1_IRQn,UART1_IRQHandler);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return SUCCESS;
}



BL_Err_Type UART_FifoConfig(UART_ID_Type uartId,UART_FifoCfg_Type *fifoCfg)

{
  undefined3 in_register_00002029;
  uint32_t uVar1;
  uint uVar2;
  uint uVar3;
  
  uVar1 = uartAddr[CONCAT31(in_register_00002029,uartId)];
  *(uint *)(uVar1 + 0x84) =
       ((fifoCfg->txFifoDmaThreshold - 1) * 0x10000 | *(uint *)(uVar1 + 0x84) & 0xffe0ffff) &
       0xe0ffffff | (fifoCfg->rxFifoDmaThreshold - 1) * 0x1000000;
  uVar3 = *(uint *)(uVar1 + 0x80) & 0xfffffffe;
  if (fifoCfg->txFifoDmaEnable == ENABLE) {
    uVar3 = *(uint *)(uVar1 + 0x80) | 1;
  }
  uVar2 = uVar3 & 0xfffffffd;
  if (fifoCfg->rxFifoDmaEnable == ENABLE) {
    uVar2 = uVar3 | 2;
  }
  *(uint *)(uVar1 + 0x80) = uVar2;
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return SUCCESS;
}



BL_Err_Type UART_Enable(UART_ID_Type uartId,UART_Direction_Type direct)

{
  undefined3 in_register_00002029;
  uint *puVar1;
  
  puVar1 = (uint *)uartAddr[CONCAT31(in_register_00002029,uartId)];
  if ((direct & ~UART_TXRX) == UART_TX) {
    *puVar1 = *puVar1 | 1;
  }
  if ((byte)(direct + ~UART_TX) < 2) {
    puVar1[1] = puVar1[1] | 1;
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return SUCCESS;
}



BL_Err_Type UART_Disable(UART_ID_Type uartId,UART_Direction_Type direct)

{
  undefined3 in_register_00002029;
  uint *puVar1;
  
  puVar1 = (uint *)uartAddr[CONCAT31(in_register_00002029,uartId)];
  if ((direct & ~UART_TXRX) == UART_TX) {
    *puVar1 = *puVar1 & 0xfffffffe;
  }
  if ((byte)(direct + ~UART_TX) < 2) {
    puVar1[1] = puVar1[1] & 0xfffffffe;
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return SUCCESS;
}



BL_Err_Type UART_SetRxTimeoutValue(UART_ID_Type uartId,uint8_t time)

{
  undefined3 in_register_00002029;
  undefined3 in_register_0000202d;
  
  *(uint *)(uartAddr[CONCAT31(in_register_00002029,uartId)] + 0x18) =
       CONCAT31(in_register_0000202d,time) - 1U |
       *(uint *)(uartAddr[CONCAT31(in_register_00002029,uartId)] + 0x18) & 0xffffff00;
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return SUCCESS;
}



BL_Err_Type UART_TxFreeRun(UART_ID_Type uartId,BL_Fun_Type txFreeRun)

{
  undefined3 in_register_00002029;
  undefined3 in_register_0000202d;
  uint *puVar1;
  
  puVar1 = (uint *)uartAddr[CONCAT31(in_register_00002029,uartId)];
  if (CONCAT31(in_register_0000202d,txFreeRun) != 1) {
    *puVar1 = *puVar1 & 0xfffffffb;
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return SUCCESS;
  }
  *puVar1 = *puVar1 | 4;
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return SUCCESS;
}



BL_Err_Type UART_AutoBaudDetection(UART_ID_Type uartId,BL_Fun_Type autoBaud)

{
  undefined3 in_register_00002029;
  undefined3 in_register_0000202d;
  uint32_t uVar1;
  
  uVar1 = uartAddr[CONCAT31(in_register_00002029,uartId)];
  if (CONCAT31(in_register_0000202d,autoBaud) != 1) {
    *(uint *)(uVar1 + 4) = *(uint *)(uVar1 + 4) & 0xfffffff7;
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return SUCCESS;
  }
  *(uint *)(uVar1 + 4) = *(uint *)(uVar1 + 4) | 8;
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return SUCCESS;
}



BL_Err_Type UART_IntMask(UART_ID_Type uartId,UART_INT_Type intType,BL_Mask_Type intMask)

{
  undefined3 in_register_00002029;
  undefined3 in_register_0000202d;
  uint uVar1;
  undefined3 in_register_00002031;
  uint uVar2;
  uint32_t uVar3;
  uint uVar4;
  
  uVar3 = uartAddr[CONCAT31(in_register_00002029,uartId)];
  uVar2 = *(uint *)(uVar3 + 0x24);
  if (CONCAT31(in_register_0000202d,intType) == 8) {
    uVar4 = 0;
    if (CONCAT31(in_register_00002031,intMask) == 1) {
      *(uint *)(uVar3 + 0x24) = uVar2 | 0xff;
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return SUCCESS;
    }
  }
  else {
    uVar1 = 1 << (intType & 0x1f);
    uVar4 = ~uVar1 & uVar2;
    if (CONCAT31(in_register_00002031,intMask) == 1) {
      *(uint *)(uVar3 + 0x24) = uVar1 | uVar2;
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return SUCCESS;
    }
  }
  *(uint *)(uVar3 + 0x24) = uVar4;
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return SUCCESS;
}



BL_Err_Type UART_IntClear(UART_ID_Type uartId,UART_INT_Type intType)

{
  undefined3 in_register_00002029;
  undefined3 in_register_0000202d;
  uint32_t uVar1;
  
  uVar1 = uartAddr[CONCAT31(in_register_00002029,uartId)];
  if (CONCAT31(in_register_0000202d,intType) != 8) {
    *(uint *)(uVar1 + 0x28) = 1 << (intType & 0x1f) | *(uint *)(uVar1 + 0x28);
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return SUCCESS;
  }
  *(uint *)(uVar1 + 0x28) = *(uint *)(uVar1 + 0x28) | 0xff;
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return SUCCESS;
}



BL_Err_Type
UART_Int_Callback_Install(UART_ID_Type uartId,UART_INT_Type intType,intCallback_Type *cbFun)

{
  undefined3 in_register_00002029;
  undefined3 in_register_0000202d;
  
  uartIntCbfArra[CONCAT31(in_register_00002029,uartId)][CONCAT31(in_register_0000202d,intType)] =
       cbFun;
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return SUCCESS;
}



BL_Err_Type UART_SendData(UART_ID_Type uartId,uint8_t *data,uint32_t len)

{
  undefined3 in_register_00002029;
  uint8_t *puVar1;
  uint uVar2;
  int iVar3;
  uint32_t uVar4;
  
  uVar4 = uartAddr[CONCAT31(in_register_00002029,uartId)];
  if (len != 0) {
    iVar3 = 160000;
    uVar2 = 0;
    do {
      puVar1 = data + uVar2;
      iVar3 = iVar3 + -1;
      if ((*(uint *)(uVar4 + 0x84) & 0x3f) == 0) {
        if (iVar3 == 0) {
          gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
          return TIMEOUT;
        }
      }
      else {
        uVar2 = uVar2 + 1;
        iVar3 = 160000;
        *(uint8_t *)(uVar4 + 0x88) = *puVar1;
      }
    } while (uVar2 < len);
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return SUCCESS;
}



uint32_t UART_ReceiveData(UART_ID_Type uartId,uint8_t *data,uint32_t maxLen)

{
  undefined3 in_register_00002029;
  uint32_t uVar1;
  uint32_t uVar2;
  uint32_t uVar3;
  
  uVar2 = uartAddr[CONCAT31(in_register_00002029,uartId)];
  uVar1 = maxLen;
  if (maxLen != 0) {
    uVar3 = 0;
    do {
      uVar1 = uVar3;
      if ((*(uint *)(uVar2 + 0x84) >> 8 & 0x3f) == 0) break;
      uVar3 = uVar1 + 1;
      data[uVar1] = *(uint8_t *)(uVar2 + 0x8c);
      uVar1 = maxLen;
    } while (maxLen != uVar3);
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return uVar1;
}



uint16_t UART_GetAutoBaudCount(UART_ID_Type uartId,UART_AutoBaudDetection_Type autoBaudDet)

{
  undefined3 in_register_00002029;
  int iVar1;
  undefined3 in_register_0000202d;
  
  iVar1 = *(int *)(uartAddr[CONCAT31(in_register_00002029,uartId)] + 0x34);
  if (CONCAT31(in_register_0000202d,autoBaudDet) != 0) {
    iVar1 = iVar1 << 0x10;
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return (uint16_t)((uint)iVar1 >> 0x10);
}



BL_Sts_Type UART_GetIntStatus(UART_ID_Type uartId,UART_INT_Type intType)

{
  undefined3 in_register_00002029;
  undefined3 in_register_0000202d;
  
  if (CONCAT31(in_register_0000202d,intType) != 8) {
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return (1 << (intType & 0x1f) &
           *(uint *)(uartAddr[CONCAT31(in_register_00002029,uartId)] + 0x20)) != 0;
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return (*(uint *)(uartAddr[CONCAT31(in_register_00002029,uartId)] + 0x20) & 0xff) != 0;
}



BL_Sts_Type UART_GetTxBusBusyStatus(UART_ID_Type uartId)

{
  undefined3 in_register_00002029;
  
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return (byte)*(undefined4 *)(uartAddr[CONCAT31(in_register_00002029,uartId)] + 0x30) & 1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type GLB_Set_UART_CLK(uint8_t enable,HBN_UART_CLK_Type clkSel,uint8_t div)

{
  undefined3 in_register_00002029;
  undefined3 in_register_00002031;
  uint uVar1;
  
  _DAT_40000008 = CONCAT31(in_register_00002031,div) | _DAT_40000008 & 0xffffffe8;
  HBN_Set_UART_CLK_Sel(clkSel);
  uVar1 = _DAT_40000008 | 0x10;
  if (CONCAT31(in_register_00002029,enable) == 0) {
    uVar1 = _DAT_40000008 & 0xffffffef;
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  _DAT_40000008 = uVar1;
  return SUCCESS;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type GLB_AHB_Slave1_Reset(BL_AHB_Slave1_Type slave1)

{
  uint uVar1;
  uint uVar2;
  
  uVar1 = 1 << (slave1 & 0x1f);
  uVar2 = ~uVar1;
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  _DAT_40000014 = uVar2 & (uVar1 | _DAT_40000014 & uVar2);
  return SUCCESS;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type GLB_UART_Sig_Swap_Set(uint8_t swapSel)

{
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  _DAT_40000080 = (uint)swapSel << 0x18 | _DAT_40000080 & 0xf8ffffff;
  return SUCCESS;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type GLB_Set_MTimer_CLK(uint8_t enable,GLB_MTIMER_CLK_Type clkSel,uint32_t div)

{
  undefined3 in_register_00002029;
  undefined3 in_register_0000202d;
  uint uVar1;
  
  uVar1 = CONCAT31(in_register_0000202d,clkSel) << 0x13 | _DAT_40000090 & 0xfff20000;
  _DAT_40000090 = uVar1 | div & 0xfffbffff;
  if (CONCAT31(in_register_00002029,enable) != 0) {
    _DAT_40000090 = uVar1 | div | 0x40000;
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return SUCCESS;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type GLB_UART_Fun_Sel(GLB_UART_SIG_Type sig,GLB_UART_SIG_FUN_Type fun)

{
  undefined3 in_register_0000202d;
  
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  _DAT_400000c0 =
       ~(0xf << ((sig & 7) << 2)) & _DAT_400000c0 |
       CONCAT31(in_register_0000202d,fun) << ((sig & 7) << 2);
  return SUCCESS;
}



uint32_t GLB_GPIO_Read(GLB_GPIO_Type gpioPin)

{
  undefined3 in_register_00002029;
  
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return (uint)((1 << (gpioPin & 0x1f) &
                *(uint *)((CONCAT31(in_register_00002029,gpioPin) >> 5) * 4 + 0x40000180)) != 0);
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void EF_Ctrl_Read_Secure_Cfg(EF_Ctrl_Sec_Param_Type *cfg)

{
  byte bVar1;
  uint uVar2;
  uint uVar3;
  
  EF_Ctrl_Load_Efuse_R0();
  uVar2 = _DAT_40007000 >> 4;
  uVar3 = _DAT_40007000 >> 0xd;
  bVar1 = (byte)(_DAT_40007000 >> 0x18);
  cfg->ef_dbg_mode = bVar1 >> 4;
  cfg->ef_dbg_jtag_0_dis = bVar1 >> 2 & 3;
  cfg->ef_sboot_en = (byte)uVar2 & 3;
  cfg->ef_no_hd_boot_en = (byte)uVar3 & 1;
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void EF_Ctrl_Read_Secure_Boot(EF_Ctrl_Sign_Type *sign,EF_Ctrl_SF_AES_Type *aes)

{
  uint uVar1;
  EF_Ctrl_SF_AES_Type EVar2;
  uint uVar3;
  
  EF_Ctrl_Load_Efuse_R0();
  uVar1 = _DAT_40007000;
  uVar3 = _DAT_40007000 >> 7;
  *sign = (EF_Ctrl_Sign_Type)(_DAT_40007000 >> 2) & EF_CTRL_SIGN_RSA;
  EVar2 = (byte)uVar1 & 3;
  if ((uVar3 & 1) == 0) {
    EVar2 = EF_CTRL_SF_AES_NONE;
  }
  *aes = EVar2;
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

uint8_t EF_Ctrl_Get_Trim_Enable(void)

{
  EF_Ctrl_Load_Efuse_R0();
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return (byte)(_DAT_40007000 >> 0xc) & 1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void EF_Ctrl_Read_Sw_Usage(uint32_t index,uint32_t *usage)

{
  EF_Ctrl_Sw_AHB_Clk_0();
  EF_Ctrl_Load_Efuse_R0();
  if (index == 0) {
    *usage = _DAT_40007010;
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type EF_Ctrl_Read_MAC_Address(uint8_t *mac)

{
  undefined4 uVar1;
  
  EF_Ctrl_Load_Efuse_R0();
  uVar1 = _DAT_40007014;
  mac[3] = (uint8_t)((uint)_DAT_40007014 >> 0x18);
  *mac = (uint8_t)uVar1;
  mac[1] = (uint8_t)((uint)uVar1 >> 8);
  mac[2] = (uint8_t)((uint)uVar1 >> 0x10);
  uVar1 = _DAT_40007018;
  mac[4] = (uint8_t)_DAT_40007018;
  mac[5] = (uint8_t)((uint)uVar1 >> 8);
  mac[6] = (uint8_t)((uint)uVar1 >> 0x10);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return SUCCESS;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type EF_Ctrl_Read_Chip_ID(uint8_t *chipID)

{
  undefined4 uVar1;
  
  chipID[6] = '\0';
  chipID[7] = '\0';
  EF_Ctrl_Load_Efuse_R0();
  uVar1 = _DAT_40007014;
  chipID[3] = (uint8_t)((uint)_DAT_40007014 >> 0x18);
  *chipID = (uint8_t)uVar1;
  chipID[1] = (uint8_t)((uint)uVar1 >> 8);
  chipID[2] = (uint8_t)((uint)uVar1 >> 0x10);
  uVar1 = _DAT_40007018;
  chipID[4] = (uint8_t)_DAT_40007018;
  chipID[5] = (uint8_t)((uint)uVar1 >> 8);
  chipID[6] = (uint8_t)((uint)uVar1 >> 0x10);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return SUCCESS;
}



void EF_Ctrl_Program_Direct_R0(uint32_t index,uint32_t *data,uint32_t len)

{
  EF_Ctrl_Sw_AHB_Clk_0();
  BL602_Delay_US(4);
  BL602_MemCpy4((uint32_t *)(&DAT_40007000 + index * 4),data,len);
  EF_Ctrl_Program_Efuse_0();
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



void EF_Ctrl_Read_Direct_R0(uint32_t index,uint32_t *data,uint32_t len)

{
  EF_Ctrl_Load_Efuse_R0();
  BL602_MemCpy4(data,(uint32_t *)(&DAT_40007000 + index * 4),len);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type HBN_Set_UART_CLK_Sel(HBN_UART_CLK_Type clkSel)

{
  undefined3 in_register_00002029;
  
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  _DAT_4000f030 = CONCAT31(in_register_00002029,clkSel) << 2 | _DAT_4000f030 & 0xfffffffb;
  return SUCCESS;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type HBN_Set_BOR_Cfg(HBN_BOR_CFG_Type *cfg)

{
  uint uVar1;
  uint uVar2;
  
  if (cfg->enableBorInt == '\0') {
    _DAT_4000f014 = _DAT_4000f014 & 0xfffbffff;
  }
  else {
    _DAT_4000f014 = _DAT_4000f014 | 0x40000;
  }
  uVar2 = (uint)cfg->borThreshold << 1;
  uVar1 = uVar2 | _DAT_4000f02c & 0xfffffffc;
  if (cfg->enablePorInBor != '\0') {
    uVar1 = uVar2 | _DAT_4000f02c & 0xfffffffd | 1;
  }
  _DAT_4000f02c = uVar1 & 0xfffffffb;
  if (cfg->enableBor != '\0') {
    _DAT_4000f02c = uVar1 | 4;
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return SUCCESS;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type
Sec_Eng_SHA256_Update
          (SEC_Eng_SHA256_Ctx *shaCtx,uint8_t *input,uint32_t len,SEC_ENG_SHA_ID_Type shaNo)

{
  byte bVar1;
  uint uVar2;
  uint uVar3;
  BL_Err_Type BVar4;
  uint32_t n;
  int iVar5;
  uint uVar6;
  
  uVar2 = _DAT_40004000;
  iVar5 = 15999999;
  do {
    iVar5 = iVar5 + -1;
    if ((_DAT_40004000 & 1) == 0) {
      uVar3 = shaCtx->total[0];
      bVar1 = shaCtx->shaFeed;
      uVar6 = uVar3 + len;
      shaCtx->total[0] = uVar6;
      uVar3 = uVar3 & 0x3f;
      if (uVar6 < len) {
        shaCtx->total[1] = shaCtx->total[1] + 1;
      }
      if (uVar3 != 0) {
        uVar6 = 0x40 - uVar3;
        n = len;
        if (len < uVar6) goto joined_r0x22016a20;
        BL602_MemCpy_Fast((void *)((int)shaCtx->shaBuf + uVar3),input,uVar6);
        _DAT_40004004 = shaCtx->shaBuf;
        _DAT_40004000 = uVar2 & 0xffbf | (uint)bVar1 << 6 | 0x10002;
        shaCtx->shaFeed = '\x01';
        input = input + uVar6;
        len = (len - 0x40) + uVar3;
      }
      n = len & 0x3f;
      uVar3 = 0;
      if (len < 0x40) goto joined_r0x22016a20;
      iVar5 = 15999999;
      goto LAB_220169e6;
    }
  } while (iVar5 != 0);
LAB_22016a26:
  BVar4 = TIMEOUT;
LAB_22016968:
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar4;
LAB_220169e6:
  uVar3 = _DAT_40004000 & 1;
  iVar5 = iVar5 + -1;
  if (uVar3 != 0) goto LAB_220169e2;
  _DAT_40004000 = (len >> 6) << 0x10 | _DAT_40004000 & 0xffbf | (uint)shaCtx->shaFeed << 6 | 2;
  _DAT_40004004 = (uint32_t *)input;
  shaCtx->shaFeed = '\x01';
  input = (uint8_t *)((int)input + (len & 0xffffffc0));
joined_r0x22016a20:
  if (n == 0) {
LAB_22016948:
    iVar5 = 15999999;
    do {
      iVar5 = iVar5 + -1;
      if ((_DAT_40004000 & 1) == 0) {
        BVar4 = SUCCESS;
        goto LAB_22016968;
      }
    } while (iVar5 != 0);
  }
  else {
    iVar5 = 15999999;
    do {
      iVar5 = iVar5 + -1;
      if ((_DAT_40004000 & 1) == 0) {
        BL602_MemCpy_Fast((void *)((int)shaCtx->shaBuf + uVar3),input,n);
        goto LAB_22016948;
      }
    } while (iVar5 != 0);
  }
  goto LAB_22016a26;
LAB_220169e2:
  if (iVar5 == 0) goto LAB_22016a26;
  goto LAB_220169e6;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address
// WARNING: Exceeded maximum restarts with more pending

void SEC_SHA_IRQHandler(void)

{
  if ((_DAT_40004000 & 0x100) != 0) {
    _DAT_40004000 = _DAT_40004000 | 0x200;
    if (secEngIntCbfArra[2] != (intCallback_Type *)0x0) {
                    // WARNING: Could not recover jumptable at 0x22016a46. Too many branches
                    // WARNING: Treating indirect jump as call
      (*secEngIntCbfArra[2])();
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return;
    }
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address
// WARNING: Exceeded maximum restarts with more pending

void SEC_TRNG_IRQHandler(void)

{
  if ((_DAT_40004200 & 0x100) != 0) {
    _DAT_40004200 = _DAT_40004200 | 0x200;
    if (secEngIntCbfArra[0] != (intCallback_Type *)0x0) {
                    // WARNING: Could not recover jumptable at 0x22016a6a. Too many branches
                    // WARNING: Treating indirect jump as call
      (*secEngIntCbfArra[0])();
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return;
    }
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void Sec_Eng_SHA256_Init(SEC_Eng_SHA256_Ctx *shaCtx,SEC_ENG_SHA_ID_Type shaNo,
                        SEC_ENG_SHA_Type shaType,uint32_t *shaTmpBuf,uint32_t *padding)

{
  undefined3 in_register_00002031;
  
  _DAT_40004000 = _DAT_40004000 & 0xffffffe3 | CONCAT31(in_register_00002031,shaType) << 2;
  memset(shaCtx,0,0x14);
  shaCtx->shaBuf = shaTmpBuf;
  shaCtx->shaPadding = padding;
  BL602_MemSet(padding,'\0',0x40);
  BL602_MemSet(shaCtx->shaPadding,0x80,1);
  Interrupt_Handler_Register(SEC_SHA_IRQn,SEC_SHA_IRQHandler);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void Sec_Eng_SHA_Start(SEC_ENG_SHA_ID_Type shaNo)

{
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  _DAT_40004000 = _DAT_40004000 & 0xffffffbf | 0x20;
  return;
}



BL_Err_Type
Sec_Eng_SHA256_Update
          (SEC_Eng_SHA256_Ctx *shaCtx,SEC_ENG_SHA_ID_Type shaNo,uint8_t *input,uint32_t len)

{
  BL_Err_Type BVar1;
  
  if (len != 0) {
    BVar1 = Sec_Eng_SHA256_Update(shaCtx,input,len,(SEC_ENG_SHA_ID_Type)len);
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return BVar1;
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return SUCCESS;
}



// WARNING: Could not reconcile some variable overlaps
// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type
Sec_Eng_SHA256_Finish(SEC_Eng_SHA256_Ctx *shaCtx,SEC_ENG_SHA_ID_Type shaNo,uint8_t *hash)

{
  undefined4 uVar1;
  uint32_t len;
  SEC_ENG_SHA_ID_Type shaNo_00;
  uint uVar2;
  int iVar3;
  uint uVar4;
  undefined uStack24;
  SEC_ENG_SHA_ID_Type SStack23;
  ushort uStack22;
  uint8_t msgLen [8];
  
  iVar3 = 15999999;
  do {
    iVar3 = iVar3 + -1;
    if ((_DAT_40004000 & 1) == 0) {
      uVar4 = shaCtx->total[0];
      uVar2 = shaCtx->total[1] << 3;
      uStack22 = (ushort)((uVar4 >> 0x1d | uVar2) << 8) | (ushort)(uVar2 >> 8) & 0xff;
      msgLen._2_2_ = (ushort)(uVar4 << 0xb) | (ushort)((uVar4 << 0x13) >> 0x18);
      uStack24 = (undefined)(uVar2 >> 0x18);
      SStack23 = (SEC_ENG_SHA_ID_Type)(uVar2 >> 0x10);
      msgLen[0] = (uint8_t)((uVar4 << 3) >> 0x18);
      msgLen[1] = (uint8_t)((uVar4 << 3) >> 0x10);
      uVar4 = uVar4 & 0x3f;
      if (uVar4 < 0x38) {
        len = 0x38 - uVar4;
      }
      else {
        len = 0x78 - uVar4;
      }
      Sec_Eng_SHA256_Update(shaCtx,(uint8_t *)shaCtx->shaPadding,len,SStack23);
      iVar3 = 15999999;
      shaNo_00 = SEC_ENG_SHA_ID0;
      do {
        iVar3 = iVar3 + -1;
        if ((_DAT_40004000 & 1) == 0) {
          BL602_MemCpy_Fast(shaCtx->shaPadding,&uStack24,8);
          Sec_Eng_SHA256_Update(shaCtx,(uint8_t *)shaCtx->shaPadding,8,shaNo_00);
          uVar1 = _DAT_40004010;
          iVar3 = 15999999;
          do {
            iVar3 = iVar3 + -1;
            if ((_DAT_40004000 & 1) == 0) {
              uVar2 = _DAT_40004000 >> 2;
              hash[1] = (uint8_t)((uint)_DAT_40004010 >> 8);
              hash[2] = (uint8_t)((uint)uVar1 >> 0x10);
              hash[3] = (uint8_t)((uint)uVar1 >> 0x18);
              *hash = (uint8_t)uVar1;
              uVar1 = _DAT_40004014;
              hash[5] = (uint8_t)((uint)_DAT_40004014 >> 8);
              hash[6] = (uint8_t)((uint)uVar1 >> 0x10);
              hash[7] = (uint8_t)((uint)uVar1 >> 0x18);
              hash[4] = (uint8_t)uVar1;
              uVar1 = _DAT_40004018;
              hash[9] = (uint8_t)((uint)_DAT_40004018 >> 8);
              hash[10] = (uint8_t)((uint)uVar1 >> 0x10);
              hash[0xb] = (uint8_t)((uint)uVar1 >> 0x18);
              hash[8] = (uint8_t)uVar1;
              uVar1 = _DAT_4000401c;
              hash[0xd] = (uint8_t)((uint)_DAT_4000401c >> 8);
              hash[0xe] = (uint8_t)((uint)uVar1 >> 0x10);
              hash[0xf] = (uint8_t)((uint)uVar1 >> 0x18);
              hash[0xc] = (uint8_t)uVar1;
              uVar1 = _DAT_40004020;
              hash[0x10] = (uint8_t)_DAT_40004020;
              hash[0x11] = (uint8_t)((uint)uVar1 >> 8);
              hash[0x12] = (uint8_t)((uint)uVar1 >> 0x10);
              hash[0x13] = (uint8_t)((uint)uVar1 >> 0x18);
              uVar1 = _DAT_40004024;
              if ((uVar2 & 6) == 0) {
                hash[0x15] = (uint8_t)((uint)_DAT_40004024 >> 8);
                hash[0x16] = (uint8_t)((uint)uVar1 >> 0x10);
                hash[0x17] = (uint8_t)((uint)uVar1 >> 0x18);
                hash[0x14] = (uint8_t)uVar1;
                uVar1 = _DAT_40004028;
                hash[0x18] = (uint8_t)_DAT_40004028;
                hash[0x19] = (uint8_t)((uint)uVar1 >> 8);
                hash[0x1a] = (uint8_t)((uint)uVar1 >> 0x10);
                hash[0x1b] = (uint8_t)((uint)uVar1 >> 0x18);
                uVar1 = _DAT_4000402c;
                if ((uVar2 & 7) == 0) {
                  hash[0x1c] = (uint8_t)_DAT_4000402c;
                  hash[0x1d] = (uint8_t)((uint)uVar1 >> 8);
                  hash[0x1e] = (uint8_t)((uint)uVar1 >> 0x10);
                  hash[0x1f] = (uint8_t)((uint)uVar1 >> 0x18);
                }
              }
              gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
              _DAT_40004000 = _DAT_40004000 & 0xffffff9f;
              return SUCCESS;
            }
          } while (iVar3 != 0);
          gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
          return TIMEOUT;
        }
      } while (iVar3 != 0);
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return TIMEOUT;
    }
  } while (iVar3 != 0);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return TIMEOUT;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type Sec_Eng_Trng_Enable(void)

{
  int iVar1;
  
  iVar1 = 15999999;
  do {
    iVar1 = iVar1 + -1;
    if ((_DAT_40004200 & 1) == 0) {
      _DAT_40004200 = _DAT_40004200 | 0x204;
      Interrupt_Handler_Register(SEC_TRNG_IRQn,SEC_TRNG_IRQHandler);
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return SUCCESS;
    }
  } while (iVar1 != 0);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  _DAT_40004200 = _DAT_40004200 | 0x204;
  return TIMEOUT;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void Sec_Eng_Trng_Disable(void)

{
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  _DAT_40004200 = _DAT_40004200 & 0xfffffffb | 0x200;
  return;
}



void bflb_platform_uart_dbg_init(uint32_t bdrate)

{
  UART_FifoCfg_Type UStack36;
  UART_FifoCfg_Type fifoCfg;
  UART_CFG_Type uart_dbg_cfg;
  
  UStack36 = (UART_FifoCfg_Type)0x1010;
  fifoCfg = 40000000;
  bflb_platform_init_uart_debug_gpio();
  GLB_Set_UART_CLK('\x01',HBN_UART_CLK_160M,'\x03');
  fifoCfg = 40000000;
  GLB_AHB_Slave1_Reset(BL_AHB_SLAVE1_UART1);
  UART_IntMask(UART1_ID,UART_INT_ALL,MASK);
  UART_Disable(UART1_ID,UART_TXRX);
  UART_Init(UART1_ID,(UART_CFG_Type *)&fifoCfg);
  UART_FifoConfig(UART1_ID,&UStack36);
  UART_TxFreeRun(UART1_ID,ENABLE);
  UART_SetRxTimeoutValue(UART1_ID,'P');
  UART_Enable(UART1_ID,UART_TXRX);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



uint32_t bflb_platform_get_log(uint8_t *data,uint32_t maxlen)

{
  uint32_t n;
  
  n = log_len;
  if (maxlen < log_len) {
    n = maxlen;
  }
  memcpy(data,eflash_loader_logbuf,n);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return n;
}



// WARNING: Variable defined which should be unmapped: ap

void bflb_platform_printf(char *fmt,...)

{
  size_t n;
  undefined4 in_a1;
  undefined4 in_a2;
  undefined4 in_a3;
  undefined4 in_a4;
  undefined4 in_a5;
  undefined4 in_a6;
  undefined4 in_a7;
  uint uVar1;
  va_list ap;
  undefined4 uStack28;
  undefined4 uStack24;
  undefined4 uStack20;
  undefined4 uStack16;
  undefined4 uStack12;
  undefined4 uStack8;
  undefined4 uStack4;
  
  if (uart_dbg_disable != '\0') {
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return;
  }
  uStack28 = in_a1;
  uStack24 = in_a2;
  uStack20 = in_a3;
  uStack16 = in_a4;
  uStack12 = in_a5;
  uStack8 = in_a6;
  uStack4 = in_a7;
  vsnprintf(bflb_platform_printf::print_buf,0x7f,fmt,&uStack28);
  n = strlen(bflb_platform_printf::print_buf);
  uVar1 = n + log_len;
  if (uVar1 < 0x1000) {
    memcpy(eflash_loader_logbuf + log_len,bflb_platform_printf::print_buf,n);
    log_len = uVar1;
  }
  UART_SendData(UART1_ID,(uint8_t *)bflb_platform_printf::print_buf,n);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void bflb_platform_clear_time(void)

{
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  _DAT_0200bff8 = 0;
  _DAT_0200bffc = 0;
  return;
}



// WARNING: Removing unreachable block (ram,0x22016f1e)
// WARNING: Globals starting with '_' overlap smaller symbols at the same address

uint64_t bflb_platform_get_time_ms(void)

{
  int iVar1;
  uint64_t in_fa0;
  UDItype UVar2;
  UDItype in_fa1;
  UDItype in_fa2;
  
  iVar1 = _DAT_0200bffc;
  GLB_Get_BCLK_Div();
  if (iVar1 != 0) {
    UVar2 = __udivdi3(in_fa1,in_fa2);
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return UVar2;
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return in_fa0;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void bflb_platform_delay_ms(uint32_t time)

{
  uint uVar1;
  int iVar2;
  uint32_t uVar3;
  int in_a1;
  uint uVar4;
  uint uVar5;
  
  uVar4 = _DAT_4000f108 >> 0xf;
  _DAT_0200bff8 = 0;
  _DAT_0200bffc = 0;
  iVar2 = 0;
  uVar1 = 0;
  uVar3 = time;
  do {
    bflb_platform_get_time_ms();
    uVar5 = uVar1 + 1;
    iVar2 = iVar2 + (uint)(uVar5 < uVar1);
    if (in_a1 != 0) {
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return;
    }
    if (time <= uVar3) {
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return;
    }
    if (iVar2 != 0) {
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return;
    }
    uVar1 = uVar5;
  } while (uVar5 <= uVar4 * time * 2);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void bflb_platform_init(uint32_t baudrate)

{
  if (init_flag != 0) {
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return;
  }
  init_flag = 1;
  clic_disable_interrupt(7);
  GLB_Set_MTimer_CLK('\x01',GLB_MTIMER_CLK_BCLK,7);
  _DAT_0200bff8 = 0;
  _DAT_0200bffc = 0;
  Sec_Eng_Trng_Enable();
  if (uart_dbg_disable != '\0') {
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return;
  }
  bflb_platform_uart_dbg_init(baudrate);
  bflb_platform_printf("system clock=%dM\r\n",_DAT_4000f108 / 1000000);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



void bflb_platform_deinit(void)

{
  if (init_flag == 0) {
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return;
  }
  init_flag = 0;
  clic_disable_interrupt(7);
  Sec_Eng_Trng_Disable();
  if (uart_dbg_disable != '\0') {
    clic_disable_interrupt(7);
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return;
  }
  UART_Disable(UART1_ID,UART_TXRX);
  GLB_AHB_Slave1_Reset(BL_AHB_SLAVE1_UART1);
  bflb_platform_deinit_uart_debug_gpio();
  clic_disable_interrupt(7);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Could not reconcile some variable overlaps

void bflb_platform_init_uart_debug_gpio(void)

{
  undefined4 uStack24;
  GLB_GPIO_Cfg_Type cfg;
  
  uStack24 = 0x10708;
  cfg._0_2_ = 0x100;
  GLB_GPIO_Init((GLB_GPIO_Cfg_Type *)&uStack24);
  GLB_UART_Sig_Swap_Set('\x02');
  GLB_UART_Fun_Sel(GLB_UART_SIG_4,GLB_UART_SIG_FUN_UART1_TXD);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Could not reconcile some variable overlaps

void bflb_platform_deinit_uart_debug_gpio(void)

{
  undefined4 uStack24;
  GLB_GPIO_Cfg_Type cfg;
  
  uStack24 = 0x2000b08;
  cfg._0_2_ = 0x100;
  GLB_GPIO_Init((GLB_GPIO_Cfg_Type *)&uStack24);
  GLB_UART_Sig_Swap_Set('\0');
  GLB_UART_Fun_Sel(GLB_UART_SIG_4,GLB_UART_SIG_FUN_UART1_RTS);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



BL_Err_Type bflb_flash_chip_erase(SPI_Flash_Cfg_Type *flashCfg)

{
  uint uVar1;
  BL_Err_Type BVar2;
  BL_Sts_Type BVar3;
  undefined3 extraout_var;
  undefined3 extraout_var_00;
  int iVar6;
  undefined auStack52 [4];
  SF_Ctrl_Cmd_Cfg_Type flashCmd;
  int iVar4;
  int iVar5;
  
  BVar2 = SFlash_Write_Enable(flashCfg);
  iVar4 = CONCAT31(extraout_var,BVar2);
  iVar6 = iVar4;
  if (iVar4 == 0) {
    BL602_MemSet4((uint32_t *)auStack52,0,5);
    auStack52[0] = '\0';
    flashCmd.nbData = (uint)flashCfg->chipEraseCmd << 0x18;
    uVar1 = 0;
    SF_Ctrl_SendCmd((SF_Ctrl_Cmd_Cfg_Type *)auStack52);
    do {
      while( true ) {
        BVar3 = SFlash_Busy(flashCfg);
        iVar5 = CONCAT31(extraout_var_00,BVar3);
        uVar1 = uVar1 + 1;
        iVar6 = iVar4;
        if (iVar5 != 1) goto LAB_22017c04;
        BL602_Delay_US(500);
        iVar6 = iVar5;
        if (uVar1 % 2000 != 0) break;
        bflb_eflash_loader_if_send_pending();
        if ((uint)flashCfg->timeCe * 3 < uVar1) goto LAB_22017c04;
      }
    } while (uVar1 <= (uint)flashCfg->timeCe * 3);
  }
LAB_22017c04:
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return (BL_Err_Type)iVar6;
}



BL_Err_Type GLB_GPIO_Init(GLB_GPIO_Cfg_Type *cfg)

{
  byte bVar1;
  uint8_t uVar2;
  uint uVar3;
  uint *puVar4;
  uint *puVar5;
  uint uVar6;
  uint uVar7;
  uint uVar8;
  
  bVar1 = cfg->gpioPin;
  puVar5 = (uint *)((uint)(bVar1 >> 5) * 4 + 0x40000190);
  uVar8 = 1 << (bVar1 & 0x1f);
  uVar3 = ~uVar8 & *puVar5;
  *puVar5 = uVar3;
  puVar4 = (uint *)((uint)(bVar1 >> 1) * 4 + 0x40000100);
  uVar7 = *puVar4;
  uVar2 = cfg->gpioMode;
  if ((bVar1 & 1) == 0) {
    uVar6 = uVar7 & 0xffffffce;
    if (uVar2 != '\x03') {
      uVar6 = uVar7 | 1;
      if (uVar2 == '\x01') {
        uVar6 = uVar7 & 0xfffffffe;
        uVar3 = uVar3 | uVar8;
      }
      uVar6 = uVar6 & 0xffffffcf;
      if (cfg->pullType == '\0') {
        uVar6 = uVar6 | 0x10;
      }
      else if (cfg->pullType == '\x01') {
        uVar6 = uVar6 | 0x20;
      }
    }
    *puVar4 = (uint)cfg->gpioFun << 8 |
              ((uint)cfg->smtCtrl << 1 | uVar6 & 0xfffffff1 | (uint)cfg->drive << 2) & 0xfffff0ff;
    *puVar5 = uVar3;
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return SUCCESS;
  }
  if (uVar2 == '\x03') {
    uVar6 = uVar7 & 0xffceffff;
  }
  else {
    uVar6 = uVar7 | 0x10000;
    if (uVar2 == '\x01') {
      uVar6 = uVar7 & 0xfffeffff;
      uVar3 = uVar3 | uVar8;
    }
    uVar6 = uVar6 & 0xffcfffff;
    if (cfg->pullType == '\0') {
      uVar6 = uVar6 | 0x100000;
    }
    else if (cfg->pullType == '\x01') {
      uVar6 = uVar6 | 0x200000;
    }
  }
  *puVar4 = (uint)cfg->gpioFun << 0x18 |
            ((uint)cfg->smtCtrl << 0x11 | (uint)cfg->drive << 0x12 | uVar6 & 0xfff1ffff) &
            0xf0ffffff;
  *puVar5 = uVar3;
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return SUCCESS;
}



BL_Err_Type GLB_GPIO_INPUT_Enable(GLB_GPIO_Type gpioPin)

{
  undefined3 in_register_00002029;
  uint *puVar1;
  
  puVar1 = (uint *)((CONCAT31(in_register_00002029,gpioPin) >> 1) * 4 + 0x40000100);
  if ((gpioPin & 1) == 0) {
    *puVar1 = *puVar1 | 1;
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return SUCCESS;
  }
  *puVar1 = *puVar1 | 0x10000;
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return SUCCESS;
}



BL_Err_Type GLB_GPIO_INPUT_Disable(GLB_GPIO_Type gpioPin)

{
  undefined3 in_register_00002029;
  uint *puVar1;
  
  puVar1 = (uint *)((CONCAT31(in_register_00002029,gpioPin) >> 1) * 4 + 0x40000100);
  if ((gpioPin & 1) == 0) {
    *puVar1 = *puVar1 & 0xfffffffe;
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return SUCCESS;
  }
  *puVar1 = *puVar1 & 0xfffeffff;
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return SUCCESS;
}



BL_Err_Type GLB_GPIO_Set_PullUp(GLB_GPIO_Type gpioPin)

{
  undefined3 in_register_00002029;
  uint *puVar1;
  
  puVar1 = (uint *)((CONCAT31(in_register_00002029,gpioPin) >> 1) * 4 + 0x40000100);
  if ((gpioPin & 1) == 0) {
    *puVar1 = *puVar1 & 0xffffffdf | 0x10;
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return SUCCESS;
  }
  *puVar1 = *puVar1 & 0xffdfffff | 0x100000;
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return SUCCESS;
}



BL_Err_Type GLB_GPIO_Set_PullDown(GLB_GPIO_Type gpioPin)

{
  undefined3 in_register_00002029;
  uint *puVar1;
  
  puVar1 = (uint *)((CONCAT31(in_register_00002029,gpioPin) >> 1) * 4 + 0x40000100);
  if ((gpioPin & 1) == 0) {
    *puVar1 = *puVar1 & 0xffffffef | 0x20;
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return SUCCESS;
  }
  *puVar1 = *puVar1 & 0xffefffff | 0x200000;
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return SUCCESS;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type AON_Power_On_BG(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017e7e. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010828)(_DAT_21010828);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type AON_Power_Off_BG(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017e88. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_2101082c)(_DAT_2101082c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type AON_Power_On_LDO11_SOC(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017e92. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010830)(_DAT_21010830);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type AON_Power_Off_LDO11_SOC(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017e9c. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010834)(_DAT_21010834);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type AON_Power_On_LDO15_RF(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017ea6. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010838)(_DAT_21010838);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type AON_Power_Off_LDO15_RF(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017eb0. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_2101083c)(_DAT_2101083c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type AON_Power_On_SFReg(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017eba. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010840)(_DAT_21010840);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type AON_Power_Off_SFReg(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017ec4. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010844)(_DAT_21010844);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type AON_LowPower_Enter_PDS0(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017ece. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010848)(_DAT_21010848);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type AON_LowPower_Exit_PDS0(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017ed8. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_2101084c)(_DAT_2101084c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void ASM_Delay_Us(uint32_t core,uint32_t cnt)

{
                    // WARNING: Could not recover jumptable at 0x22017ee2. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010850)(_DAT_21010850);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void BL602_Delay_US(uint32_t cnt)

{
                    // WARNING: Could not recover jumptable at 0x22017eec. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010854)(_DAT_21010854);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void BL602_Delay_MS(uint32_t cnt)

{
                    // WARNING: Could not recover jumptable at 0x22017ef6. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010858)(_DAT_21010858);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void * BL602_MemCpy(void *dst,void *src,uint32_t n)

{
  void *pvVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017f00. Too many branches
                    // WARNING: Treating indirect jump as call
  pvVar1 = (void *)(*_DAT_2101085c)(_DAT_2101085c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return pvVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

uint32_t * BL602_MemCpy4(uint32_t *dst,uint32_t *src,uint32_t n)

{
  uint32_t *puVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017f0a. Too many branches
                    // WARNING: Treating indirect jump as call
  puVar1 = (uint32_t *)(*_DAT_21010860)(_DAT_21010860);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return puVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void * BL602_MemCpy_Fast(void *pdst,void *psrc,uint32_t n)

{
  void *pvVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017f14. Too many branches
                    // WARNING: Treating indirect jump as call
  pvVar1 = (void *)(*_DAT_21010864)(_DAT_21010864);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return pvVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void * BL602_MemSet(void *s,uint8_t c,uint32_t n)

{
  void *pvVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017f1e. Too many branches
                    // WARNING: Treating indirect jump as call
  pvVar1 = (void *)(*_DAT_21010868)(_DAT_21010868);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return pvVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

uint32_t * BL602_MemSet4(uint32_t *dst,uint32_t val,uint32_t n)

{
  uint32_t *puVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017f28. Too many branches
                    // WARNING: Treating indirect jump as call
  puVar1 = (uint32_t *)(*_DAT_2101086c)(_DAT_2101086c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return puVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

int BL602_MemCmp(void *s1,void *s2,uint32_t n)

{
  int iVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017f32. Too many branches
                    // WARNING: Treating indirect jump as call
  iVar1 = (*_DAT_21010870)(_DAT_21010870);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return iVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void EF_Ctrl_Sw_AHB_Clk_0(void)

{
                    // WARNING: Could not recover jumptable at 0x22017f3c. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010874)(_DAT_21010874);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void EF_Ctrl_Program_Efuse_0(void)

{
                    // WARNING: Could not recover jumptable at 0x22017f46. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010878)(_DAT_21010878);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void EF_Ctrl_Load_Efuse_R0(void)

{
                    // WARNING: Could not recover jumptable at 0x22017f50. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_2101087c)(_DAT_2101087c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Sts_Type EF_Ctrl_Busy(void)

{
  BL_Sts_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017f5a. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010880)(_DAT_21010880);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Sts_Type EF_Ctrl_AutoLoad_Done(void)

{
  BL_Sts_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017f64. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010884)(_DAT_21010884);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void EF_Ctrl_Clear(uint32_t index,uint32_t len)

{
                    // WARNING: Could not recover jumptable at 0x22017f6e. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010894)(_DAT_21010894);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type GLB_SW_System_Reset(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017f78. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210108bc)(_DAT_210108bc);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type GLB_SW_CPU_Reset(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017f82. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210108c0)(_DAT_210108c0);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type GLB_SW_POR_Reset(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017f8c. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210108c4)(_DAT_210108c4);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type GLB_Select_Internal_Flash(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017f96. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210108c8)(_DAT_210108c8);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type GLB_Select_External_Flash(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017fa0. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210108cc)(_DAT_210108cc);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type GLB_Deswap_Flash_Pin(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017faa. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210108d0)(_DAT_210108d0);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type GLB_Swap_Flash_Pin(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017fb4. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210108d4)(_DAT_210108d4);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type GLB_GPIO_OUTPUT_Enable(GLB_GPIO_Type gpioPin)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017fbe. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210108dc)(_DAT_210108dc);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type GLB_GPIO_OUTPUT_Disable(GLB_GPIO_Type gpioPin)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017fc8. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210108e0)(_DAT_210108e0);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type GLB_GPIO_Set_HZ(GLB_GPIO_Type gpioPin)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017fd2. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210108e4)(_DAT_210108e4);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

uint8_t GLB_GPIO_Get_Fun(GLB_GPIO_Type gpioPin)

{
  uint8_t uVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017fdc. Too many branches
                    // WARNING: Treating indirect jump as call
  uVar1 = (*_DAT_210108e8)(_DAT_210108e8);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return uVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void HBN_Power_Down_Flash(SPI_Flash_Cfg_Type *flashCfg)

{
                    // WARNING: Could not recover jumptable at 0x22017fe6. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_210108f0)(_DAT_210108f0);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void HBN_Enable(uint8_t aGPIOIeCfg,HBN_LDO_LEVEL_Type ldoLevel,HBN_LEVEL_Type hbnLevel)

{
                    // WARNING: Could not recover jumptable at 0x22017ff0. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_210108f4)(_DAT_210108f4);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type HBN_Reset(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22017ffa. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210108f8)(_DAT_210108f8);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type HBN_Set_Ldo11_Aon_Vout(HBN_LDO_LEVEL_Type ldoLevel)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018004. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210108fc)(_DAT_210108fc);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type HBN_Set_Ldo11_Rt_Vout(HBN_LDO_LEVEL_Type ldoLevel)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x2201800e. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010900)(_DAT_21010900);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type HBN_Set_Ldo11_Soc_Vout(HBN_LDO_LEVEL_Type ldoLevel)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018018. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010904)(_DAT_21010904);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type HBN_Hw_Pu_Pd_Cfg(uint8_t enable)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018022. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010924)(_DAT_21010924);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type HBN_Pin_WakeUp_Mask(uint8_t maskVal)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x2201802c. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010928)(_DAT_21010928);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type
HBN_GPIO7_Dbg_Pull_Cfg(BL_Fun_Type pupdEn,BL_Fun_Type iesmtEn,BL_Fun_Type dlyEn,uint8_t dlySec)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018036. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_2101092c)(_DAT_2101092c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type HBN_Set_Embedded_Flash_Pullup(uint8_t enable)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018040. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010930)(_DAT_21010930);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type L1C_Set_Wrap(BL_Fun_Type wrap)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x2201804a. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010934)(_DAT_21010934);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type L1C_Set_Way_Disable(uint8_t disableVal)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018054. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010938)(_DAT_21010938);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type L1C_IROM_2T_Access_Set(uint8_t enable)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x2201805e. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_2101093c)(_DAT_2101093c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type PDS_Reset(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018068. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010940)(_DAT_21010940);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type PDS_Enable(PDS_CTL_Type *cfg,PDS_CTL4_Type *cfg4,uint32_t pdsSleepCnt)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018072. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010944)(_DAT_21010944);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type PDS_Force_Config(PDS_CTL2_Type *cfg2,PDS_CTL3_Type *cfg3)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x2201807c. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010948)(_DAT_21010948);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type PDS_RAM_Config(PDS_RAM_CFG_Type *ramCfg)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018086. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_2101094c)(_DAT_2101094c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type
PDS_Default_Level_Config
          (PDS_DEFAULT_LV_CFG_Type *defaultLvCfg,PDS_RAM_CFG_Type *ramCfg,uint32_t pdsSleepCnt)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018090. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010950)(_DAT_21010950);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SEC_Eng_Turn_On_Sec_Ring(void)

{
                    // WARNING: Could not recover jumptable at 0x2201809a. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010978)(_DAT_21010978);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SEC_Eng_Turn_Off_Sec_Ring(void)

{
                    // WARNING: Could not recover jumptable at 0x220180a4. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_2101097c)(_DAT_2101097c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SFlash_Init(SF_Ctrl_Cfg_Type *pSfCtrlCfg)

{
                    // WARNING: Could not recover jumptable at 0x220180ae. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010980)(_DAT_21010980);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type SFlash_SetSPIMode(SF_Ctrl_Mode_Type mode)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x220180b8. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010984)(_DAT_21010984);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type
SFlash_Read_Reg(SPI_Flash_Cfg_Type *flashCfg,uint8_t regIndex,uint8_t *regValue,uint8_t regLen)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x220180c2. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010988)(_DAT_21010988);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type
SFlash_Write_Reg(SPI_Flash_Cfg_Type *flashCfg,uint8_t regIndex,uint8_t *regValue,uint8_t regLen)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x220180cc. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_2101098c)(_DAT_2101098c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Sts_Type SFlash_Busy(SPI_Flash_Cfg_Type *flashCfg)

{
  BL_Sts_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x220180d6. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010990)(_DAT_21010990);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type SFlash_Write_Enable(SPI_Flash_Cfg_Type *flashCfg)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x220180e0. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010994)(_DAT_21010994);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type SFlash_Qspi_Enable(SPI_Flash_Cfg_Type *flashCfg)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x220180ea. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010998)(_DAT_21010998);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SFlash_Volatile_Reg_Write_Enable(SPI_Flash_Cfg_Type *flashCfg)

{
                    // WARNING: Could not recover jumptable at 0x220180f4. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_2101099c)(_DAT_2101099c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type SFlash_Chip_Erase(SPI_Flash_Cfg_Type *flashCfg)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x220180fe. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210109a0)(_DAT_210109a0);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type SFlash_Sector_Erase(SPI_Flash_Cfg_Type *flashCfg,uint32_t secNum)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018108. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210109a4)(_DAT_210109a4);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type SFlash_Blk32_Erase(SPI_Flash_Cfg_Type *flashCfg,uint32_t blkNum)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018112. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210109a8)(_DAT_210109a8);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type SFlash_Blk64_Erase(SPI_Flash_Cfg_Type *flashCfg,uint32_t blkNum)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x2201811c. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210109ac)(_DAT_210109ac);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type SFlash_Erase(SPI_Flash_Cfg_Type *flashCfg,uint32_t startaddr,uint32_t endaddr)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018126. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210109b0)(_DAT_210109b0);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type
SFlash_Program(SPI_Flash_Cfg_Type *flashCfg,SF_Ctrl_IO_Type ioMode,uint32_t addr,uint8_t *data,
              uint32_t len)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018130. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210109b4)(_DAT_210109b4);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SFlash_GetUniqueId(uint8_t *data,uint8_t idLen)

{
                    // WARNING: Could not recover jumptable at 0x2201813a. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_210109b8)(_DAT_210109b8);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SFlash_GetJedecId(SPI_Flash_Cfg_Type *flashCfg,uint8_t *data)

{
                    // WARNING: Could not recover jumptable at 0x22018144. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_210109bc)(_DAT_210109bc);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SFlash_GetDeviceId(uint8_t *data)

{
                    // WARNING: Could not recover jumptable at 0x2201814e. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_210109c0)(_DAT_210109c0);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SFlash_Powerdown(void)

{
                    // WARNING: Could not recover jumptable at 0x22018158. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_210109c4)(_DAT_210109c4);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SFlash_Releae_Powerdown(SPI_Flash_Cfg_Type *flashCfg)

{
                    // WARNING: Could not recover jumptable at 0x22018162. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_210109c8)(_DAT_210109c8);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SFlash_SetBurstWrap(SPI_Flash_Cfg_Type *flashCfg)

{
                    // WARNING: Could not recover jumptable at 0x2201816c. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_210109cc)(_DAT_210109cc);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SFlash_DisableBurstWrap(SPI_Flash_Cfg_Type *flashCfg)

{
                    // WARNING: Could not recover jumptable at 0x22018176. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_210109d0)(_DAT_210109d0);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type SFlash_Software_Reset(SPI_Flash_Cfg_Type *flashCfg)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018180. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210109d4)(_DAT_210109d4);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SFlash_Reset_Continue_Read(SPI_Flash_Cfg_Type *flashCfg)

{
                    // WARNING: Could not recover jumptable at 0x2201818a. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_210109d8)(_DAT_210109d8);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type
SFlash_Set_IDbus_Cfg
          (SPI_Flash_Cfg_Type *flashCfg,SF_Ctrl_IO_Type ioMode,uint8_t contRead,uint32_t addr,
          uint32_t len)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018194. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210109dc)(_DAT_210109dc);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type
SFlash_IDbus_Read_Enable(SPI_Flash_Cfg_Type *flashCfg,SF_Ctrl_IO_Type ioMode,uint8_t contRead)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x2201819e. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210109e0)(_DAT_210109e0);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type SFlash_Cache_Enable_Set(uint8_t wayDisable)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x220181a8. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210109e4)(_DAT_210109e4);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type SFlash_Cache_Flush(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x220181b2. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210109e8)(_DAT_210109e8);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type
SFlash_Cache_Read_Enable
          (SPI_Flash_Cfg_Type *flashCfg,SF_Ctrl_IO_Type ioMode,uint8_t contRead,uint8_t wayDisable)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x220181bc. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210109ec)(_DAT_210109ec);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SFlash_Cache_Hit_Count_Get(uint32_t *hitCountLow,uint32_t *hitCountHigh)

{
                    // WARNING: Could not recover jumptable at 0x220181c6. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_210109f0)(_DAT_210109f0);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

uint32_t SFlash_Cache_Miss_Count_Get(void)

{
  uint32_t uVar1;
  
                    // WARNING: Could not recover jumptable at 0x220181d0. Too many branches
                    // WARNING: Treating indirect jump as call
  uVar1 = (*_DAT_210109f4)(_DAT_210109f4);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return uVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SFlash_Cache_Read_Disable(void)

{
                    // WARNING: Could not recover jumptable at 0x220181da. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_210109f8)(_DAT_210109f8);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type
SFlash_Read(SPI_Flash_Cfg_Type *flashCfg,SF_Ctrl_IO_Type ioMode,uint8_t contRead,uint32_t addr,
           uint8_t *data,uint32_t len)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x220181e4. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210109fc)(_DAT_210109fc);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type
SFlash_Read_Reg_With_Cmd
          (SPI_Flash_Cfg_Type *flashCfg,uint8_t readRegCmd,uint8_t *regValue,uint8_t regLen)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x220181ee. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010a00)(_DAT_21010a00);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type
SFlash_Write_Reg_With_Cmd
          (SPI_Flash_Cfg_Type *flashCfg,uint8_t writeRegCmd,uint8_t *regValue,uint8_t regLen)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x220181f8. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010a04)(_DAT_21010a04);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SF_Cfg_Init_Ext_Flash_Gpio(uint8_t extFlashPin)

{
                    // WARNING: Could not recover jumptable at 0x22018202. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010a0c)(_DAT_21010a0c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SF_Cfg_Init_Internal_Flash_Gpio(void)

{
                    // WARNING: Could not recover jumptable at 0x2201820c. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010a10)(_DAT_21010a10);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SF_Cfg_Deinit_Ext_Flash_Gpio(uint8_t extFlashPin)

{
                    // WARNING: Could not recover jumptable at 0x22018216. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010a14)(_DAT_21010a14);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SF_Cfg_Restore_GPIO17_Fun(uint8_t fun)

{
                    // WARNING: Could not recover jumptable at 0x22018220. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010a18)(_DAT_21010a18);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type SF_Cfg_Get_Flash_Cfg_Need_Lock(uint32_t flashID,SPI_Flash_Cfg_Type *pFlashCfg)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x2201822a. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010a1c)(_DAT_21010a1c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SF_Cfg_Init_Flash_Gpio(uint8_t flashPinCfg,uint8_t restoreDefault)

{
                    // WARNING: Could not recover jumptable at 0x22018234. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010a20)(_DAT_21010a20);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

uint32_t SF_Cfg_Flash_Identify
                   (uint8_t callFromFlash,uint32_t autoScan,uint32_t flashPinCfg,
                   uint8_t restoreDefault,SPI_Flash_Cfg_Type *pFlashCfg)

{
  undefined3 in_register_00002029;
  uint32_t uVar1;
  
                    // WARNING: Could not recover jumptable at 0x2201823e. Too many branches
                    // WARNING: Treating indirect jump as call
  uVar1 = (*_DAT_21010a24)(CONCAT31(in_register_00002029,callFromFlash),_DAT_21010a24);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return uVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SF_Ctrl_Enable(SF_Ctrl_Cfg_Type *cfg)

{
                    // WARNING: Could not recover jumptable at 0x22018248. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010a28)(_DAT_21010a28);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SF_Ctrl_Select_Pad(SF_Ctrl_Pad_Sel sel)

{
                    // WARNING: Could not recover jumptable at 0x22018252. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010a2c)(_DAT_21010a2c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SF_Ctrl_Set_Owner(SF_Ctrl_Owner_Type owner)

{
                    // WARNING: Could not recover jumptable at 0x2201825c. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010a30)(_DAT_21010a30);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SF_Ctrl_Disable(void)

{
                    // WARNING: Could not recover jumptable at 0x22018266. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010a34)(_DAT_21010a34);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SF_Ctrl_AES_Enable_BE(void)

{
                    // WARNING: Could not recover jumptable at 0x22018270. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010a38)(_DAT_21010a38);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SF_Ctrl_AES_Enable_LE(void)

{
                    // WARNING: Could not recover jumptable at 0x2201827a. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010a3c)(_DAT_21010a3c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SF_Ctrl_AES_Set_Region
               (uint8_t region,uint8_t enable,uint8_t hwKey,uint32_t startAddr,uint32_t endAddr,
               uint8_t locked)

{
                    // WARNING: Could not recover jumptable at 0x22018284. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010a40)(_DAT_21010a40);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SF_Ctrl_AES_Set_Key(uint8_t region,uint8_t *key,SF_Ctrl_AES_Key_Type keyType)

{
                    // WARNING: Could not recover jumptable at 0x2201828e. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010a44)(_DAT_21010a44);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SF_Ctrl_AES_Set_Key_BE(uint8_t region,uint8_t *key,SF_Ctrl_AES_Key_Type keyType)

{
                    // WARNING: Could not recover jumptable at 0x22018298. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010a48)(_DAT_21010a48);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SF_Ctrl_AES_Set_IV(uint8_t region,uint8_t *iv,uint32_t addrOffset)

{
                    // WARNING: Could not recover jumptable at 0x220182a2. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010a4c)(_DAT_21010a4c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SF_Ctrl_AES_Set_IV_BE(uint8_t region,uint8_t *iv,uint32_t addrOffset)

{
                    // WARNING: Could not recover jumptable at 0x220182ac. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010a50)(_DAT_21010a50);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SF_Ctrl_AES_Enable(void)

{
                    // WARNING: Could not recover jumptable at 0x220182b6. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010a54)(_DAT_21010a54);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SF_Ctrl_AES_Disable(void)

{
                    // WARNING: Could not recover jumptable at 0x220182c0. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010a58)(_DAT_21010a58);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SF_Ctrl_Set_Flash_Image_Offset(uint32_t addrOffset)

{
                    // WARNING: Could not recover jumptable at 0x220182ca. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010a5c)(_DAT_21010a5c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

uint32_t SF_Ctrl_Get_Flash_Image_Offset(void)

{
  uint32_t uVar1;
  
                    // WARNING: Could not recover jumptable at 0x220182d4. Too many branches
                    // WARNING: Treating indirect jump as call
  uVar1 = (*_DAT_21010a60)(_DAT_21010a60);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return uVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SF_Ctrl_Select_Clock(SF_Ctrl_Sahb_Type sahbType)

{
                    // WARNING: Could not recover jumptable at 0x220182de. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010a64)(_DAT_21010a64);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SF_Ctrl_SendCmd(SF_Ctrl_Cmd_Cfg_Type *cfg)

{
                    // WARNING: Could not recover jumptable at 0x220182e8. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010a68)(_DAT_21010a68);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SF_Ctrl_Icache_Set(SF_Ctrl_Cmd_Cfg_Type *cfg,uint8_t cmdValid)

{
                    // WARNING: Could not recover jumptable at 0x220182f2. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010a6c)(_DAT_21010a6c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SF_Ctrl_Icache2_Set(SF_Ctrl_Cmd_Cfg_Type *cfg,uint8_t cmdValid)

{
                    // WARNING: Could not recover jumptable at 0x220182fc. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010a70)(_DAT_21010a70);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Sts_Type SF_Ctrl_GetBusyState(void)

{
  BL_Sts_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018306. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010a74)(_DAT_21010a74);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

uint8_t SF_Ctrl_Is_AES_Enable(void)

{
  uint8_t uVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018310. Too many branches
                    // WARNING: Treating indirect jump as call
  uVar1 = (*_DAT_21010a78)(_DAT_21010a78);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return uVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

uint8_t SF_Ctrl_Get_Clock_Delay(void)

{
  uint8_t uVar1;
  
                    // WARNING: Could not recover jumptable at 0x2201831a. Too many branches
                    // WARNING: Treating indirect jump as call
  uVar1 = (*_DAT_21010a7c)(_DAT_21010a7c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return uVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void SF_Ctrl_Set_Clock_Delay(uint8_t delay)

{
                    // WARNING: Could not recover jumptable at 0x22018324. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010a80)(_DAT_21010a80);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type XIP_SFlash_State_Save(SPI_Flash_Cfg_Type *pFlashCfg,uint32_t *offset)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x2201832e. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010a84)(_DAT_21010a84);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type XIP_SFlash_State_Restore(SPI_Flash_Cfg_Type *pFlashCfg,uint32_t offset)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018338. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010a88)(_DAT_21010a88);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type
XIP_SFlash_Erase_Need_Lock(SPI_Flash_Cfg_Type *pFlashCfg,uint32_t startaddr,uint32_t endaddr)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018342. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010a8c)(_DAT_21010a8c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type
XIP_SFlash_Write_Need_Lock(SPI_Flash_Cfg_Type *pFlashCfg,uint32_t addr,uint8_t *data,uint32_t len)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x2201834c. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010a90)(_DAT_21010a90);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type
XIP_SFlash_Read_Need_Lock(SPI_Flash_Cfg_Type *pFlashCfg,uint32_t addr,uint8_t *data,uint32_t len)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018356. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010a94)(_DAT_21010a94);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type XIP_SFlash_GetJedecId_Need_Lock(SPI_Flash_Cfg_Type *pFlashCfg,uint8_t *data)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018360. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010a98)(_DAT_21010a98);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type XIP_SFlash_GetDeviceId_Need_Lock(SPI_Flash_Cfg_Type *pFlashCfg,uint8_t *data)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x2201836a. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010a9c)(_DAT_21010a9c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type
XIP_SFlash_GetUniqueId_Need_Lock(SPI_Flash_Cfg_Type *pFlashCfg,uint8_t *data,uint8_t idLen)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018374. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010aa0)(_DAT_21010aa0);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type XIP_SFlash_Read_Via_Cache_Need_Lock(uint32_t addr,uint8_t *data,uint32_t len)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x2201837e. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010aa4)(_DAT_21010aa4);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

int XIP_SFlash_Read_With_Lock(SPI_Flash_Cfg_Type *pFlashCfg,uint32_t addr,uint8_t *dst,int len)

{
  int iVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018388. Too many branches
                    // WARNING: Treating indirect jump as call
  iVar1 = (*_DAT_21010aa8)(_DAT_21010aa8);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return iVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

int XIP_SFlash_Write_With_Lock(SPI_Flash_Cfg_Type *pFlashCfg,uint32_t addr,uint8_t *src,int len)

{
  int iVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018392. Too many branches
                    // WARNING: Treating indirect jump as call
  iVar1 = (*_DAT_21010aac)(_DAT_21010aac);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return iVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

int XIP_SFlash_Erase_With_Lock(SPI_Flash_Cfg_Type *pFlashCfg,uint32_t addr,int len)

{
  int iVar1;
  
                    // WARNING: Could not recover jumptable at 0x2201839c. Too many branches
                    // WARNING: Treating indirect jump as call
  iVar1 = (*_DAT_21010ab0)(_DAT_21010ab0);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return iVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void XIP_SFlash_Opt_Enter(uint8_t *aesEnable)

{
                    // WARNING: Could not recover jumptable at 0x220183a6. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010ab4)(_DAT_21010ab4);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void XIP_SFlash_Opt_Exit(uint8_t aesEnable)

{
                    // WARNING: Could not recover jumptable at 0x220183b0. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010ab8)(_DAT_21010ab8);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

uint32_t BFLB_Soft_CRC32(void *dataIn,uint32_t len)

{
  uint32_t uVar1;
  
                    // WARNING: Could not recover jumptable at 0x220183ba. Too many branches
                    // WARNING: Treating indirect jump as call
  uVar1 = (*_DAT_21010abc)(_DAT_21010abc);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return uVar1;
}



// WARNING: Variable defined which should be unmapped: tmp

BL_Err_Type SFlash_Restore_From_Powerdown(SPI_Flash_Cfg_Type *pFlashCfg,uint8_t flashContRead)

{
  SF_Ctrl_IO_Type SVar1;
  BL_Err_Type BVar2;
  undefined3 in_register_0000202d;
  undefined4 uStack28;
  uint32_t jdecId;
  uint8_t tmp [8];
  
  SVar1 = pFlashCfg->ioMode;
  uStack28 = 0;
  SFlash_Releae_Powerdown(pFlashCfg);
  BL602_Delay_US(0x78);
  SVar1 = SVar1 & 0xf;
  SFlash_GetJedecId(pFlashCfg,(uint8_t *)&uStack28);
  if ((SVar1 + ~SF_CTRL_DO_MODE & 0xfd) == 0) {
    SFlash_Qspi_Enable(pFlashCfg);
  }
  if ((pFlashCfg->ioMode >> 4 & 1) == 0) {
    L1C_Set_Wrap(ENABLE);
    SFlash_Write_Enable(pFlashCfg);
    SFlash_SetBurstWrap(pFlashCfg);
  }
  else {
    L1C_Set_Wrap(DISABLE);
  }
  if (CONCAT31(in_register_0000202d,flashContRead) != 0) {
    SFlash_Read(pFlashCfg,SVar1,'\x01',0,(uint8_t *)&jdecId,8);
    BVar2 = SFlash_Set_IDbus_Cfg(pFlashCfg,SVar1,'\x01',0,0x20);
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return BVar2;
  }
  BVar2 = SFlash_Set_IDbus_Cfg(pFlashCfg,SVar1,'\0',0,0x20);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar2;
}



// WARNING: Variable defined which should be unmapped: tempVal

BL_Err_Type
SFlash_RCV_Enable(SPI_Flash_Cfg_Type *pFlashCfg,uint8_t rCmd,uint8_t wCmd,uint8_t bitPos)

{
  int iVar1;
  BL_Sts_Type BVar2;
  BL_Err_Type BVar3;
  undefined3 extraout_var;
  uint uVar4;
  undefined3 extraout_var_00;
  undefined3 extraout_var_01;
  uint uStack36;
  uint32_t tempVal;
  
  uStack36 = 0;
  iVar1 = 0;
  do {
    BVar2 = SFlash_Busy(pFlashCfg);
    if (CONCAT31(extraout_var,BVar2) != 1) {
      SFlash_Read_Reg_With_Cmd(pFlashCfg,rCmd,(uint8_t *)&uStack36,'\x01');
      uVar4 = 0;
      if ((uStack36 >> (bitPos & 0x1f) & 1) != 0) goto LAB_220184c2;
      uStack36 = 1 << (bitPos & 0x1f) | uStack36;
      SFlash_Write_Enable(pFlashCfg);
      BVar3 = SFlash_Write_Reg_With_Cmd(pFlashCfg,wCmd,(uint8_t *)&uStack36,'\x01');
      uVar4 = CONCAT31(extraout_var_00,BVar3);
      if (uVar4 == 0) goto LAB_22018506;
      goto LAB_220184c2;
    }
    iVar1 = iVar1 + 1;
    BL602_Delay_US(500);
  } while (iVar1 != 0xea61);
  goto LAB_22018532;
  while (BL602_Delay_US(500), iVar1 != 0xea61) {
LAB_22018506:
    BVar2 = SFlash_Busy(pFlashCfg);
    iVar1 = iVar1 + 1;
    if (CONCAT31(extraout_var_01,BVar2) != 1) {
      SFlash_Read_Reg_With_Cmd(pFlashCfg,rCmd,(uint8_t *)&uStack36,'\x01');
      uVar4 = ~(uStack36 >> (bitPos & 0x1f)) & 1;
      goto LAB_220184c2;
    }
  }
LAB_22018532:
  uVar4 = 1;
LAB_220184c2:
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return (BL_Err_Type)uVar4;
}



BL_Err_Type
SFlash_Erase_Security_Register(SPI_Flash_Cfg_Type *pFlashCfg,SFlash_Sec_Reg_Cfg *pSecRegCfg)

{
  int iVar1;
  uint uVar2;
  int iVar3;
  BL_Sts_Type BVar4;
  BL_Err_Type BVar5;
  undefined3 extraout_var;
  undefined3 extraout_var_00;
  undefined3 extraout_var_01;
  undefined3 extraout_var_02;
  uint8_t uVar6;
  undefined auStack52 [4];
  SF_Ctrl_Cmd_Cfg_Type flashCmd;
  
  uVar6 = pSecRegCfg->enterSecOptCmd;
  if (uVar6 == '\0') {
LAB_2201858a:
    BVar5 = SFlash_Write_Enable(pFlashCfg);
    iVar1 = CONCAT31(extraout_var_00,BVar5);
    if (iVar1 == 0) {
      BL602_MemSet4((uint32_t *)auStack52,0,5);
      flashCmd.nbData = (uint)pSecRegCfg->eraseCmd << 0x18 | (uint)pSecRegCfg->blockNum << 0xc;
      auStack52[0] = '\0';
      auStack52[3] = '\x03';
      uVar2 = 0;
      SF_Ctrl_SendCmd((SF_Ctrl_Cmd_Cfg_Type *)auStack52);
      do {
        BVar4 = SFlash_Busy(pFlashCfg);
        uVar2 = uVar2 + 1;
        if (CONCAT31(extraout_var_01,BVar4) != 1) {
          if (uVar6 == '\0') goto LAB_22018594;
          BL602_MemSet4((uint32_t *)auStack52,0,5);
          flashCmd.nbData = (uint)pSecRegCfg->exitSecOptCmd << 0x18;
          auStack52[0] = '\x01';
          iVar3 = 800000;
          SF_Ctrl_SendCmd((SF_Ctrl_Cmd_Cfg_Type *)auStack52);
          goto LAB_22018632;
        }
        BL602_Delay_US(500);
      } while (uVar2 <= (uint)pFlashCfg->timeEsector * 3);
      iVar1 = 1;
    }
    goto LAB_22018594;
  }
  BL602_MemSet4((uint32_t *)auStack52,0,5);
  flashCmd.nbData = (uint)pSecRegCfg->enterSecOptCmd << 0x18;
  auStack52[0] = '\x01';
  iVar1 = 800000;
  SF_Ctrl_SendCmd((SF_Ctrl_Cmd_Cfg_Type *)auStack52);
  do {
    BVar4 = SF_Ctrl_GetBusyState();
    iVar1 = iVar1 + -1;
    if (CONCAT31(extraout_var,BVar4) != 1) {
      uVar6 = '\x01';
      goto LAB_2201858a;
    }
  } while (iVar1 != 0);
  goto LAB_22018642;
  while (iVar3 != 0) {
LAB_22018632:
    BVar4 = SF_Ctrl_GetBusyState();
    iVar3 = iVar3 + -1;
    if (CONCAT31(extraout_var_02,BVar4) != 1) goto LAB_22018594;
  }
LAB_22018642:
  iVar1 = 2;
LAB_22018594:
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return (BL_Err_Type)iVar1;
}



BL_Err_Type
SFlash_Program_Security_Register(SPI_Flash_Cfg_Type *pFlashCfg,SFlash_Sec_Reg_Cfg *pSecRegCfg)

{
  uint8_t uVar1;
  byte bVar2;
  byte bVar3;
  int iVar4;
  BL_Sts_Type BVar5;
  BL_Err_Type BVar6;
  undefined3 extraout_var;
  undefined3 extraout_var_00;
  undefined3 extraout_var_01;
  undefined3 extraout_var_02;
  uint uVar7;
  uint n;
  uint uVar8;
  uint8_t *psrc;
  uint uVar9;
  uint uVar10;
  undefined auStack84 [4];
  SF_Ctrl_Cmd_Cfg_Type flashCmd;
  
  uVar1 = pSecRegCfg->enterSecOptCmd;
  psrc = pSecRegCfg->data;
  uVar8 = pSecRegCfg->addr;
  uVar10 = pSecRegCfg->len;
  if (uVar1 == '\0') {
    BL602_MemSet4((uint32_t *)(SF_Ctrl_Cmd_Cfg_Type *)auStack84,0,5);
    bVar2 = pSecRegCfg->programCmd;
    if (uVar10 == 0) {
      iVar4 = 0;
      goto LAB_220187a4;
    }
LAB_220186d8:
    auStack84[3] = '\x03';
    auStack84[0] = '\x01';
    uVar9 = 0;
    do {
      BVar6 = SFlash_Write_Enable(pFlashCfg);
      iVar4 = CONCAT31(extraout_var_00,BVar6);
      if (iVar4 != 0) goto LAB_220187a4;
      uVar7 = 0x100 - (uVar8 & 0xff);
      n = uVar10 - uVar9;
      if (uVar7 < uVar10 - uVar9) {
        n = uVar7;
      }
      bVar3 = pSecRegCfg->blockNum;
      BL602_MemCpy_Fast((void *)0x4000b700,psrc,n);
      flashCmd.nbData = (uint)bVar3 << 0xc | (uint)bVar2 << 0x18 | uVar8;
      uVar8 = uVar8 + n;
      flashCmd._4_4_ = n;
      SF_Ctrl_SendCmd((SF_Ctrl_Cmd_Cfg_Type *)auStack84);
      psrc = psrc + n;
      uVar7 = 0;
      while( true ) {
        BVar5 = SFlash_Busy(pFlashCfg);
        iVar4 = CONCAT31(extraout_var_01,BVar5);
        uVar7 = uVar7 + 1;
        if (iVar4 != 1) break;
        BL602_Delay_US(100);
        if ((uint)pFlashCfg->timePagePgm * 0x14 < uVar7) goto LAB_220187a4;
      }
      uVar9 = uVar9 + n;
    } while (uVar9 < uVar10);
    if (uVar1 == '\0') {
LAB_220187a2:
      iVar4 = 0;
      goto LAB_220187a4;
    }
LAB_2201876c:
    BL602_MemSet4((uint32_t *)(SF_Ctrl_Cmd_Cfg_Type *)auStack84,0,5);
    flashCmd.nbData = (uint)pSecRegCfg->exitSecOptCmd << 0x18;
    auStack84[0] = '\x01';
    iVar4 = 800000;
    SF_Ctrl_SendCmd((SF_Ctrl_Cmd_Cfg_Type *)auStack84);
    do {
      BVar5 = SF_Ctrl_GetBusyState();
      iVar4 = iVar4 + -1;
      if (CONCAT31(extraout_var_02,BVar5) != 1) goto LAB_220187a2;
    } while (iVar4 != 0);
  }
  else {
    BL602_MemSet4((uint32_t *)(SF_Ctrl_Cmd_Cfg_Type *)auStack84,0,5);
    flashCmd.nbData = (uint)pSecRegCfg->enterSecOptCmd << 0x18;
    auStack84[0] = '\x01';
    iVar4 = 800000;
    SF_Ctrl_SendCmd((SF_Ctrl_Cmd_Cfg_Type *)auStack84);
    do {
      BVar5 = SF_Ctrl_GetBusyState();
      iVar4 = iVar4 + -1;
      if (CONCAT31(extraout_var,BVar5) != 1) {
        BL602_MemSet4((uint32_t *)(SF_Ctrl_Cmd_Cfg_Type *)auStack84,0,5);
        auStack84[3] = '\x03';
        auStack84[0] = '\x01';
        uVar1 = '\x01';
        bVar2 = pSecRegCfg->programCmd;
        if (uVar10 != 0) goto LAB_220186d8;
        goto LAB_2201876c;
      }
    } while (iVar4 != 0);
  }
  iVar4 = 2;
LAB_220187a4:
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return (BL_Err_Type)iVar4;
}



BL_Err_Type SFlash_Read_Security_Register(SFlash_Sec_Reg_Cfg *pSecRegCfg)

{
  uint8_t uVar1;
  byte bVar2;
  int iVar3;
  uint32_t n;
  BL_Sts_Type BVar4;
  undefined3 extraout_var;
  undefined3 extraout_var_00;
  undefined3 extraout_var_01;
  uint uVar5;
  uint uVar6;
  uint8_t *pdst;
  uint uVar7;
  undefined auStack84 [4];
  SF_Ctrl_Cmd_Cfg_Type flashCmd;
  
  uVar1 = pSecRegCfg->enterSecOptCmd;
  pdst = pSecRegCfg->data;
  uVar6 = pSecRegCfg->addr;
  uVar7 = pSecRegCfg->len;
  if (uVar1 == '\0') {
    BL602_MemSet4((uint32_t *)(SF_Ctrl_Cmd_Cfg_Type *)auStack84,0,5);
    bVar2 = pSecRegCfg->readCmd;
    if (uVar7 == 0) {
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return SUCCESS;
    }
LAB_22018876:
    flashCmd.rwFlag = '\x01';
    auStack84[3] = '\x03';
    auStack84[0] = '\0';
    uVar5 = 0;
    do {
      n = uVar7 - uVar5;
      flashCmd.nbData = (uint)pSecRegCfg->blockNum << 0xc | (uint)bVar2 << 0x18 | uVar6;
      if (n < 0x100) {
        flashCmd._4_4_ = n + 3 & 0xfffffffc;
      }
      else {
        flashCmd._4_4_ = 0x100;
        n = 0x100;
      }
      iVar3 = 800000;
      SF_Ctrl_SendCmd((SF_Ctrl_Cmd_Cfg_Type *)auStack84);
      while( true ) {
        BVar4 = SF_Ctrl_GetBusyState();
        iVar3 = iVar3 + -1;
        if (CONCAT31(extraout_var_00,BVar4) != 1) break;
        if (iVar3 == 0) goto LAB_220188e4;
      }
      uVar5 = uVar5 + n;
      BL602_MemCpy_Fast(pdst,(void *)0x4000b700,n);
      uVar6 = uVar6 + n;
      pdst = pdst + n;
    } while (uVar5 < uVar7);
    if (uVar1 == '\0') {
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return SUCCESS;
    }
LAB_2201892c:
    BL602_MemSet4((uint32_t *)(SF_Ctrl_Cmd_Cfg_Type *)auStack84,0,5);
    flashCmd.nbData = (uint)pSecRegCfg->exitSecOptCmd << 0x18;
    auStack84[0] = '\x01';
    iVar3 = 800000;
    SF_Ctrl_SendCmd((SF_Ctrl_Cmd_Cfg_Type *)auStack84);
    do {
      BVar4 = SF_Ctrl_GetBusyState();
      iVar3 = iVar3 + -1;
      if (CONCAT31(extraout_var_01,BVar4) != 1) {
        gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
        return SUCCESS;
      }
    } while (iVar3 != 0);
  }
  else {
    BL602_MemSet4((uint32_t *)(SF_Ctrl_Cmd_Cfg_Type *)auStack84,0,5);
    flashCmd.nbData = (uint)pSecRegCfg->enterSecOptCmd << 0x18;
    auStack84[0] = '\x01';
    iVar3 = 800000;
    SF_Ctrl_SendCmd((SF_Ctrl_Cmd_Cfg_Type *)auStack84);
    do {
      BVar4 = SF_Ctrl_GetBusyState();
      iVar3 = iVar3 + -1;
      if (CONCAT31(extraout_var,BVar4) != 1) {
        BL602_MemSet4((uint32_t *)(SF_Ctrl_Cmd_Cfg_Type *)auStack84,0,5);
        auStack84[3] = '\x03';
        auStack84[0] = '\0';
        flashCmd.rwFlag = '\x01';
        uVar1 = '\x01';
        bVar2 = pSecRegCfg->readCmd;
        if (uVar7 == 0) goto LAB_2201892c;
        goto LAB_22018876;
      }
    } while (iVar3 != 0);
  }
LAB_220188e4:
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return TIMEOUT;
}



// WARNING: Variable defined which should be unmapped: regValue
// WARNING: Could not reconcile some variable overlaps

BL_Err_Type SFlash_Clear_Status_Register(SPI_Flash_Cfg_Type *pFlashCfg)

{
  byte bVar1;
  uint uVar2;
  BL_Err_Type BVar3;
  undefined3 extraout_var;
  undefined3 extraout_var_00;
  int iVar4;
  undefined2 uStack22;
  uint8_t readRegValue0;
  uint8_t readRegValue1;
  int iStack20;
  uint32_t regValue;
  
  bVar1 = pFlashCfg->ioMode;
  iStack20 = 0;
  uStack22 = 0;
  SFlash_Read_Reg(pFlashCfg,'\0',(uint8_t *)&uStack22,'\x01');
  SFlash_Read_Reg(pFlashCfg,'\x01',(uint8_t *)((int)&uStack22 + 1),'\x01');
  iVar4 = 0;
  if ((~(1 << ((uint)pFlashCfg->qeIndex * 8 + (uint)pFlashCfg->qeBit & 0x1f) |
         1 << ((uint)pFlashCfg->busyIndex * 8 + (uint)pFlashCfg->busyBit & 0x1f) |
        1 << ((uint)pFlashCfg->wrEnableIndex * 8 + (uint)pFlashCfg->wrEnableBit & 0x1f)) &
      (uint)uStack22) == 0) {
LAB_220189e8:
    gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
    return (BL_Err_Type)iVar4;
  }
  BVar3 = SFlash_Write_Enable(pFlashCfg);
  iVar4 = CONCAT31(extraout_var,BVar3);
  if (iVar4 == 0) {
    uVar2 = (uint)(((bVar1 & 0xf) - 2 & 0xfd) == 0);
    if (pFlashCfg->qeWriteRegLen == '\x02') {
      iStack20 = uVar2 << ((uint)pFlashCfg->qeIndex * 8 + (uint)pFlashCfg->qeBit & 0x1f);
      SFlash_Write_Reg(pFlashCfg,'\0',&stack0xffffffec,'\x02');
      goto LAB_220189e8;
    }
    iStack20 = 0;
    if (pFlashCfg->qeIndex == 0) {
      iStack20 = uVar2 << (pFlashCfg->qeBit & 0x1f);
    }
    SFlash_Write_Reg(pFlashCfg,'\0',&stack0xffffffec,'\x01');
    BVar3 = SFlash_Write_Enable(pFlashCfg);
    if (CONCAT31(extraout_var_00,BVar3) == 0) {
      iStack20 = 0;
      if (pFlashCfg->qeIndex == '\x01') {
        iStack20 = uVar2 << (pFlashCfg->qeBit & 0x1f);
      }
      SFlash_Write_Reg(pFlashCfg,'\x01',&stack0xffffffec,'\x01');
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return BVar3;
    }
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return ERROR;
}



// WARNING: Could not reconcile some variable overlaps

BL_Err_Type SF_Cfg_Get_Flash_Cfg_Need_Lock_Ext(uint32_t flashID,SPI_Flash_Cfg_Type *pFlashCfg)

{
  BL_Err_Type BVar1;
  int iVar2;
  undefined3 extraout_var;
  uint32_t uVar3;
  Flash_Info_t *pFVar4;
  uint8_t auStack108 [4];
  uint8_t buf [92];
  
  if (flashID == 0) {
    XIP_SFlash_Read_Via_Cache_Need_Lock(0x23000008,auStack108,0x5c);
    iVar2 = BL602_MemCmp(auStack108,&DAT_22017bc8,4);
    if ((iVar2 == 0) && (uVar3 = BFLB_Soft_CRC32(buf,0x54), buf._84_4_ == uVar3)) {
      BL602_MemCpy_Fast(pFlashCfg,buf,0x54);
      iVar2 = 0;
    }
    else {
LAB_22018ac8:
      iVar2 = 1;
    }
  }
  else {
    BVar1 = SF_Cfg_Get_Flash_Cfg_Need_Lock(flashID,pFlashCfg);
    iVar2 = CONCAT31(extraout_var,BVar1);
    if (iVar2 != 0) {
      pFVar4 = flashInfos;
      iVar2 = 0;
      if (flashID != 0x1440a1) {
        do {
          pFVar4 = pFVar4 + 1;
          iVar2 = iVar2 + 1;
          if (iVar2 == 0x18) goto LAB_22018ac8;
        } while (flashID != pFVar4->jedecID);
      }
      BL602_MemCpy_Fast(pFlashCfg,flashInfos[iVar2].cfg,0x54);
      gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
      return SUCCESS;
    }
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return (BL_Err_Type)iVar2;
}



uint32_t SF_Cfg_Flash_Identify_Ext
                   (uint8_t callFromFlash,uint32_t autoScan,uint32_t flashPinCfg,
                   uint8_t restoreDefault,SPI_Flash_Cfg_Type *pFlashCfg)

{
  undefined3 in_register_00002029;
  uint uVar1;
  int iVar2;
  Flash_Info_t *pFVar3;
  
  uVar1 = SF_Cfg_Flash_Identify(callFromFlash,autoScan,flashPinCfg,restoreDefault,pFlashCfg);
  if (CONCAT31(in_register_00002029,callFromFlash) != 0) {
    SFlash_Set_IDbus_Cfg(pFlashCfg,pFlashCfg->ioMode & 0xf,'\x01',0,0x20);
  }
  if (-1 < (int)uVar1) {
    pFVar3 = flashInfos;
    uVar1 = uVar1 & 0xffffff;
    iVar2 = 0;
    if (uVar1 != 0x1440a1) {
      do {
        pFVar3 = pFVar3 + 1;
        iVar2 = iVar2 + 1;
        if (iVar2 == 0x18) {
          gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
          return uVar1;
        }
      } while (uVar1 != pFVar3->jedecID);
    }
    BL602_MemCpy_Fast(pFlashCfg,flashInfos[iVar2].cfg,0x54);
    uVar1 = uVar1 | 0x80000000;
  }
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return uVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type GLB_Set_System_CLK_Div(uint8_t hclkDiv,uint8_t bclkDiv)

{
  undefined3 in_register_00002029;
  undefined3 in_register_0000202d;
  
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  _DAT_40000000 =
       (CONCAT31(in_register_00002029,hclkDiv) << 8 | _DAT_40000000 & 0xffff00ff) & 0xff00ffff |
       CONCAT31(in_register_0000202d,bclkDiv) << 0x10 | 0xc;
  _DAT_40000ffc = 0;
  return SUCCESS;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type AON_Power_On_MBG(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018efa. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010810)(_DAT_21010810);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type AON_Power_Off_MBG(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018f04. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010814)(_DAT_21010814);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type AON_Power_On_XTAL(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018f0e. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010818)(_DAT_21010818);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type AON_Set_Xtal_CapCode(uint8_t capIn,uint8_t capOut)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018f18. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_2101081c)(_DAT_2101081c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

uint8_t AON_Get_Xtal_CapCode(void)

{
  uint8_t uVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018f22. Too many branches
                    // WARNING: Treating indirect jump as call
  uVar1 = (*_DAT_21010820)(_DAT_21010820);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return uVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type AON_Power_Off_XTAL(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018f2c. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010824)(_DAT_21010824);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

uint8_t EF_Ctrl_Get_Trim_Parity(uint32_t val,uint8_t len)

{
  uint8_t uVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018f36. Too many branches
                    // WARNING: Treating indirect jump as call
  uVar1 = (*_DAT_21010888)(_DAT_21010888);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return uVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void EF_Ctrl_Read_RC32M_Trim(Efuse_Ana_RC32M_Trim_Type *trim)

{
                    // WARNING: Could not recover jumptable at 0x22018f40. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_2101088c)(_DAT_2101088c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

void EF_Ctrl_Read_RC32K_Trim(Efuse_Ana_RC32K_Trim_Type *trim)

{
                    // WARNING: Could not recover jumptable at 0x22018f4a. Too many branches
                    // WARNING: Treating indirect jump as call
  (*_DAT_21010890)(_DAT_21010890);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

GLB_ROOT_CLK_Type GLB_Get_Root_CLK_Sel(void)

{
  GLB_ROOT_CLK_Type GVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018f54. Too many branches
                    // WARNING: Treating indirect jump as call
  GVar1 = (*_DAT_21010898)(_DAT_21010898);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return GVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

uint8_t GLB_Get_BCLK_Div(void)

{
  uint8_t uVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018f5e. Too many branches
                    // WARNING: Treating indirect jump as call
  uVar1 = (*_DAT_210108a0)(_DAT_210108a0);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return uVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

uint8_t GLB_Get_HCLK_Div(void)

{
  uint8_t uVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018f68. Too many branches
                    // WARNING: Treating indirect jump as call
  uVar1 = (*_DAT_210108a4)(_DAT_210108a4);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return uVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type Update_SystemCoreClockWith_XTAL(GLB_PLL_XTAL_Type xtalType)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018f72. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210108a8)(_DAT_210108a8);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type GLB_Set_System_CLK(GLB_PLL_XTAL_Type xtalType,GLB_SYS_CLK_Type clkFreq)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018f7c. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210108ac)(_DAT_210108ac);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type System_Core_Clock_Update_From_RC32M(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018f86. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210108b0)(_DAT_210108b0);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type GLB_Set_SF_CLK(uint8_t enable,GLB_SFLASH_CLK_Type clkSel,uint8_t div)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018f90. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210108b4)(_DAT_210108b4);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type GLB_Set_PKA_CLK_Sel(GLB_PKA_CLK_Type clkSel)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018f9a. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_210108b8)(_DAT_210108b8);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type HBN_32K_Sel(HBN_32K_CLK_Type clkType)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018fa4. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010908)(_DAT_21010908);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type HBN_Set_ROOT_CLK_Sel(HBN_ROOT_CLK_Type rootClk)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018fae. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_2101090c)(_DAT_2101090c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type HBN_Power_On_Xtal_32K(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018fb8. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010910)(_DAT_21010910);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type HBN_Power_Off_Xtal_32K(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018fc2. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010914)(_DAT_21010914);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type HBN_Trim_RC32K(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018fcc. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010920)(_DAT_21010920);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type PDS_Trim_RC32M(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018fd6. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010954)(_DAT_21010954);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type PDS_Select_RC32M_As_PLL_Ref(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018fe0. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010958)(_DAT_21010958);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type PDS_Select_XTAL_As_PLL_Ref(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018fea. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_2101095c)(_DAT_2101095c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type PDS_Power_On_PLL(PDS_PLL_XTAL_Type xtalType)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018ff4. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010960)(_DAT_21010960);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type PDS_Enable_PLL_All_Clks(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22018ffe. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010964)(_DAT_21010964);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type PDS_Disable_PLL_All_Clks(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22019008. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010968)(_DAT_21010968);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type PDS_Enable_PLL_Clk(PDS_PLL_CLK_Type pllClk)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22019012. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_2101096c)(_DAT_2101096c);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type PDS_Disable_PLL_Clk(PDS_PLL_CLK_Type pllClk)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x2201901c. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010970)(_DAT_21010970);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}



// WARNING: Globals starting with '_' overlap smaller symbols at the same address

BL_Err_Type PDS_Power_Off_PLL(void)

{
  BL_Err_Type BVar1;
  
                    // WARNING: Could not recover jumptable at 0x22019026. Too many branches
                    // WARNING: Treating indirect jump as call
  BVar1 = (*_DAT_21010974)(_DAT_21010974);
  gp = (uint32_t *)((int)eflash_loader_readbuf + 0x6d4);
  return BVar1;
}


