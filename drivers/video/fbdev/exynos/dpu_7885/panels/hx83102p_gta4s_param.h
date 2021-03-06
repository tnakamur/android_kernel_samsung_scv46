#ifndef __HX83102P_PARAM_H__
#define __HX83102P_PARAM_H__
#include <linux/types.h>
#include <linux/kernel.h>

#define EXTEND_BRIGHTNESS	306
#define UI_MAX_BRIGHTNESS	255
#define UI_DEFAULT_BRIGHTNESS	128

struct lcd_seq_info {
	const unsigned char	*cmd;
	unsigned int	len;
	unsigned int	sleep;
};

#if defined(CONFIG_SEC_AOT)
extern int aot_enabled;
#endif

static const unsigned char SEQ_SET_B9_PW[] = {
	0xB9,
	0x83, 0x10, 0x2E,
};

static const unsigned char SEQ_SET_E9_OTP_SETTING[] = {
	0xE9,
	0xCD,
};

static const unsigned char SEQ_SET_BB_OTP_SETTING[] = {
	0xBB,
	0x01,
};

static const unsigned char SEQ_SET_E9_OTP_SETTING2[] = {
	0xE9,
	0x00,
};

static const unsigned char SEQ_HX83102P_BL[] = {
	0x51,
	0x00, 0x00,
};

static const unsigned char SEQ_HX83102P_BLON[] = {
	0x53,
	0x24,
};

static const unsigned char SEQ_HX83102P_BL_PWM_PREQ[] = {
	0xC9,
	0x04, 0x5D, 0xC0,
};

static const unsigned char SEQ_SET_CABC_PRIORITY_SETTING[] = {
	0xCE,
	0x00, 0x50, 0xF0,
};

static const unsigned char SEQ_SET_D2_VOP[] = {
	0xD2,
	0x29, 0x29,
};

static const unsigned char SEQ_HX83102P_BL_PAD[] = {
	0xD9,
	0x00, 0x01, 0x02,
};

static const unsigned char SEQ_SET_D5_GIP_FORWARD[] = {
	0xD5,
	0x03, 0x02, 0x03, 0x02, 0x18, 0x18, 0x18, 0x18,
	0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19,
	0x24, 0x24, 0x24, 0x24, 0x18, 0x18, 0x18, 0x18,
	0x01, 0x00, 0x01, 0x00, 0x07, 0x06, 0x07, 0x06,
	0x05, 0x04, 0x05, 0x04, 0x21, 0x20, 0x21, 0x20,
	0x23, 0x22, 0x23, 0x22,
};

static const unsigned char SEQ_SET_D6_GIP_BACKWARD[] = {
	0xD6,
	0x04, 0x05, 0x04, 0x05, 0x18, 0x18, 0x18, 0x18,
	0x19, 0x19, 0x19, 0x19, 0x18, 0x18, 0x18, 0x18,
	0x24, 0x24, 0x24, 0x24, 0x19, 0x19, 0x19, 0x19,
	0x06, 0x07, 0x06, 0x07, 0x00, 0x01, 0x00, 0x01,
	0x02, 0x03, 0x02, 0x03, 0x22, 0x23, 0x22, 0x23,
	0x20, 0x21, 0x20, 0x21,
};

static const unsigned char SEQ_SET_B4_TIMING[] = {
	0xB4,
	0x98, 0x74, 0x01, 0x01, 0x98, 0x74, 0x68, 0x50,
	0x0F, 0xA0, 0x01, 0x58, 0x00, 0xFF, 0x00, 0xFF,
};

static const unsigned char SEQ_SET_D3_GIP0[] = {
	0xD3,
	0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x00, 0x08,
	0x08, 0x27, 0x27, 0x22, 0x2F, 0x11, 0x11, 0x04,
	0x04, 0x32, 0x10, 0x0F, 0x00, 0x0F, 0x32, 0x10,
	0x0D, 0x00, 0x0D, 0x32, 0x17, 0x9B, 0x07, 0x9B,
	0x00, 0x00, 0x21, 0x2F, 0x01, 0x00, 0x21, 0x38,
	0x01, 0x55,
};

static const unsigned char SEQ_SET_D1_TP_CTRL[] = {
	0xD1,
	0x37, 0x0C, 0xFF,
};

static const unsigned char SEQ_SET_B1_POWER[] = {
	0xB1,
	0x10, 0xFA, 0xAF, 0xAF, 0x29, 0x29, 0xB2, 0x57,
	0x4D, 0x36, 0x36, 0x36, 0x36, 0x22, 0x21, 0x15,
	0x00,
};

static const unsigned char SEQ_SET_B2_RESOLUTION[] = {
	0xB2,
	0x00, 0xB0, 0x47, 0x80, 0x00, 0x14, 0x44, 0x2C,
};

static const unsigned char SEQ_SET_E0_ANALOG_GAMMA[] = {
	0xE0,
	0x00, 0x04, 0x0B, 0x11, 0x17, 0x25, 0x3D, 0x43,
	0x4B, 0x48, 0x64, 0x6C, 0x74, 0x82, 0x81, 0x88,
	0x9B, 0xAE, 0xAD, 0x56, 0x5D, 0x68, 0x73, 0x00,
	0x04, 0x0B, 0x11, 0x17, 0x25, 0x3D, 0x43, 0x4B,
	0x48, 0x64, 0x6C, 0x74, 0x89, 0x8A, 0x99, 0x9B,
	0xAE, 0xAD, 0x56, 0x5D, 0x68, 0x73,
};

static const unsigned char SEQ_SET_C0_GAMMA_OPT[] = {
	0xC0,
	0x23, 0x23, 0x22, 0x11, 0xA2, 0x17, 0x00, 0x80,
	0x00, 0x00, 0x08, 0x00, 0x63, 0x63,
};

static const unsigned char SEQ_SET_CC_NORMALLY_BLACK[] = {
	0xCC,
	0x02,
};

static const unsigned char SEQ_SET_C8_CP[] = {
	0xC8,
	0x00, 0x04, 0x04, 0x00, 0x00, 0x82,
};

static const unsigned char SEQ_SET_BF_GAS[] = {
	0xBF,
	0xFC, 0x85, 0x80,
};

static const unsigned char SEQ_SET_D0_CASCADE[] = {
	0xD0,
	0x07, 0x04, 0x05,
};

static const unsigned char SEQ_SET_BD_BANK01[] = {
	0xBD,
	0x01,
};

static const unsigned char SEQ_SET_D3_GIP1[] = {
	0xD3,
	0x01, 0x00, 0xFC, 0x00, 0x00, 0x11, 0x10, 0x00,
	0x0E,
};

static const unsigned char SEQ_SET_BD_BANK02[] = {
	0xBD,
	0x02,
};

static const unsigned char SEQ_SET_B4_GAMMA_BIAS[] = {
	0xB4,
	0x4E,
};

static const unsigned char SEQ_SET_BF_VCOM[] = {
	0xBF,
	0xF2,
};

static const unsigned char SEQ_SET_D8_GIP[] = {
	0xD8,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xF0,
};

static const unsigned char SEQ_SET_BD_BANK00[] = {
	0xBD,
	0x00,
};

static const unsigned char SEQ_SET_BD_BANK03[] = {
	0xBD,
	0x03,
};

static const unsigned char SEQ_SET_E7_TOUCH_BANK0[] = {
	0xE7,
	0x12, 0x13, 0x02, 0x02, 0x2B, 0x2B, 0x0E, 0x0E,
	0x00, 0x14, 0x28, 0x79, 0x1A, 0x78, 0x01, 0x01,
	0x00, 0x00, 0x00, 0x00, 0x17, 0x00,
};

static const unsigned char SEQ_SET_E7_TOUCH_BANK1[] = {
	0xE7,
	0x02, 0x24, 0x01, 0x8F, 0x0D, 0xB2, 0x0E,
};

static const unsigned char SEQ_SET_E7_TOUCH_BANK2[] = {
	0xE7,
	0xFF, 0x01, 0xFD, 0x01, 0x00, 0x00, 0x22, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x81, 0x00, 0x02, 0x40,
};

static const unsigned char SEQ_SET_C1_DGC_ON[] = {
	0xC1,
	0x01,
};

static const unsigned char SEQ_SET_C1_DGC_GAMMA1[] = {
	0xC1,
	0x00, 0x04, 0x07, 0x0B, 0x0F, 0x12, 0x16, 0x19,
	0x1D, 0x21, 0x24, 0x28, 0x2B, 0x2E, 0x31, 0x35,
	0x38, 0x3C, 0x3F, 0x43, 0x49, 0x50, 0x57, 0x5F,
	0x66, 0x6D, 0x75, 0x7D, 0x84, 0x8C, 0x94, 0x9D,
	0xA4, 0xAD, 0xB5, 0xBD, 0xC6, 0xCF, 0xD8, 0xE0,
	0xEA, 0xEE, 0xF1, 0xF3, 0xF5, 0xF7, 0x0F, 0xA7,
	0x98, 0xBC, 0xDC, 0xAC, 0x78, 0xEC, 0xCA, 0x43,
	0x61, 0x80,
};

static const unsigned char SEQ_SET_C1_DGC_GAMMA2[] = {
	0xC1,
	0x00, 0x04, 0x07, 0x0B, 0x0F, 0x12, 0x16, 0x1A,
	0x1E, 0x21, 0x25, 0x28, 0x2C, 0x2F, 0x32, 0x36,
	0x3A, 0x3D, 0x41, 0x44, 0x4A, 0x52, 0x59, 0x60,
	0x68, 0x6F, 0x77, 0x7F, 0x87, 0x8F, 0x97, 0x9F,
	0xA7, 0xB0, 0xB8, 0xC0, 0xC9, 0xD2, 0xDB, 0xE4,
	0xED, 0xF2, 0xF4, 0xF7, 0xF9, 0xFB, 0x0F, 0xFD,
	0x36, 0x69, 0x21, 0xCA, 0x39, 0x05, 0x81, 0x91,
	0x9D, 0x80,
};

static const unsigned char SEQ_SET_C1_DGC_GAMMA3[] = {
	0xC1,
	0x00, 0x04, 0x07, 0x0C, 0x10, 0x13, 0x17, 0x1B,
	0x1F, 0x22, 0x26, 0x29, 0x2D, 0x30, 0x33, 0x37,
	0x3B, 0x3E, 0x42, 0x45, 0x4C, 0x53, 0x5B, 0x62,
	0x6A, 0x71, 0x79, 0x81, 0x88, 0x91, 0x99, 0xA1,
	0xA9, 0xB2, 0xB9, 0xC2, 0xCB, 0xD4, 0xDC, 0xE6,
	0xF0, 0xF5, 0xF7, 0xFA, 0xFD, 0xFF, 0x0C, 0x28,
	0x33, 0x1A, 0x76, 0x72, 0x24, 0xC5, 0x4C, 0x8D,
	0x0E, 0x80,
};

static const unsigned char SEQ_SET_HX83102P_CABC_ON[] = {
	0x55,
	0x01,
};

static const unsigned char SEQ_SET_E4_CABC_BANK0[] = {
	0xE4,
	0x2D, 0x41, 0x2C, 0x99, 0xBF, 0xE6, 0x0C, 0x33,
	0x66, 0x99, 0xCC, 0xFF, 0xEA, 0xFF, 0x03, 0x1E,
	0x1E, 0x1E, 0x1E, 0x00, 0x00, 0x05, 0x01, 0x14,
};

static const unsigned char SEQ_SET_E4_CABC_BANK1[] = {
	0xE4,
	0xE1, 0xE1, 0xE1, 0xE1, 0xE1, 0xE1, 0xE1, 0xE1,
	0xC7, 0xB2, 0xA0, 0x90, 0x81, 0x75, 0x69, 0x5F,
	0x55, 0x4C, 0x44, 0x3D, 0x36, 0x2E, 0x29, 0x21,
	0x1B, 0x15, 0x10, 0x0D, 0x0C, 0x0C, 0x0C, 0x54,
	0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
};

static const unsigned char SEQ_SET_B9_CLOSE_PW[] = {
	0xB9,
	0x00, 0x00, 0x00,
};

static const unsigned char SEQ_SLEEP_OUT[] = {
	0x11,
};

static const unsigned char SEQ_DISPLAY_ON[] = {
	0x29,
};

static const unsigned char SEQ_DISPLAY_OFF[] = {
	0x28,
	0x00, 0x00,
};

static const unsigned char SEQ_SLEEP_IN[] = {
	0x10,
	0x00, 0x00,
};

static const unsigned char SEQ_SET_HX83102P_CABC_OFF[] = {
	0x55,
	0x00,
};

static const unsigned char SEQ_SET_E9_VSOM_SOFT1[] = {
	0xE9,
	0xFF,
};

static const unsigned char SEQ_SET_BF_VSOM_SOFT[] = {
	0xBF,
	0x02,
};

static const unsigned char SEQ_SET_E9_VSOM_SOFT2[] = {
	0xE9,
	0x00,
};

/* platform brightness <-> bl reg */
static unsigned int brightness_table[EXTEND_BRIGHTNESS + 1] = {
	0,
	0, 0, 29, 40, 51, 63, 74, 86, 97, 108, /* 3: 29 */
	120, 131, 143, 154, 165, 177, 188, 200, 211, 223,
	234, 245, 257, 268, 280, 291, 302, 314, 325, 337,
	348, 359, 371, 382, 394, 405, 417, 428, 439, 451,
	462, 474, 485, 496, 508, 519, 531, 542, 553, 565,
	576, 588, 599, 611, 622, 633, 645, 656, 668, 679,
	690, 702, 713, 725, 736, 747, 759, 770, 782, 793,
	805, 816, 827, 839, 850, 862, 873, 884, 896, 907,
	919, 930, 941, 953, 964, 976, 987, 999, 1010, 1021,
	1033, 1044, 1056, 1067, 1078, 1090, 1101, 1113, 1124, 1135,
	1147, 1158, 1170, 1181, 1193, 1204, 1215, 1227, 1238, 1250,
	1261, 1272, 1284, 1295, 1307, 1318, 1330, 1340, 1351, 1361, /* 117: 1330 */
	1372, 1382, 1393, 1403, 1414, 1424, 1435, 1445, 1456, 1467,
	1477, 1488, 1498, 1509, 1519, 1530, 1540, 1551, 1561, 1572,
	1583, 1593, 1604, 1614, 1625, 1635, 1646, 1656, 1667, 1677,
	1688, 1699, 1709, 1720, 1730, 1741, 1751, 1762, 1772, 1783,
	1793, 1804, 1815, 1825, 1836, 1846, 1857, 1867, 1878, 1888,
	1899, 1909, 1920, 1930, 1941, 1952, 1962, 1973, 1983, 1994,
	2004, 2015, 2025, 2036, 2046, 2057, 2068, 2078, 2089, 2099,
	2110, 2120, 2131, 2141, 2152, 2162, 2173, 2184, 2194, 2205,
	2215, 2226, 2236, 2247, 2257, 2268, 2278, 2289, 2300, 2310,
	2321, 2331, 2342, 2352, 2363, 2373, 2384, 2394, 2405, 2415,
	2426, 2437, 2447, 2458, 2468, 2479, 2489, 2500, 2510, 2521,
	2531, 2542, 2553, 2563, 2574, 2584, 2595, 2605, 2616, 2626,
	2637, 2647, 2658, 2669, 2679, 2690, 2700, 2711, 2721, 2732,
	2742, 2753, 2763, 2774, 2785, 2785, 2785, 2785, 2785, 2785, /* 255: 2785 */
	2785, 2785, 2785, 2785, 2785, 2785, 2785, 2785, 2785, 2785,
	2785, 2785, 2785, 2785, 2785, 2785, 2785, 2785, 2785, 2785,
	2785, 2785, 2785, 2785, 2785, 2785, 2785, 2785, 2785, 2785,
	2785, 2785, 2785, 2785, 2785, 2785, 2785, 2785, 2785, 2785,
	2785, 2785, 2785, 2785, 2785, 3770,
};
#endif /* __HX83102P_PARAM_H__ */
