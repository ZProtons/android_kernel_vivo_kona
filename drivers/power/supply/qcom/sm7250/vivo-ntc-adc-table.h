/*
 * vivo ntc temp  param header file
 *
 */
struct adc_map_temp {
	int temp;
	unsigned long voltage;
};

/*BAT_BTB_ADC/PCB_BTB_ADC*/
static const struct adc_map_temp adc_map_temp_table1[] = {
{-300, 9999999},
{-290, 3224672},
{-280, 3219720},
{-270, 3214498},
{-260, 3208925},
{-250, 3203123},
{-240, 3196944},
{-230, 3190466},
{-220, 3183646},
{-210, 3176414},
{-200, 3168782},
{-190, 3160897},
{-180, 3152426},
{-170, 3143649},
{-160, 3134401},
{-150, 3124706},
{-140, 3114540},
{-130, 3103894},
{-120, 3092730},
{-110, 3081087},
{-100, 3068887},
{-90, 3056132},
{-80, 3042871},
{-70, 3028988},
{-60, 3014522},
{-50, 2999477},
{-40, 2983792},
{-30, 2967524},
{-20, 2950597},
{-10, 2932986},
{00, 2914754},
{10, 2895883},
{20, 2876345},
{30, 2856096},
{40, 2835211},
{50, 2813488},
{60, 2791142},
{70, 2768289},
{80, 2744484},
{90, 2720187},
{100, 2695086},
{110, 2669256},
{120, 2642797},
{130, 2615836},
{140, 2588206},
{150, 2559666},
{160, 2530655},
{170, 2501339},
{180, 2471032},
{190, 2440244},
{200, 2408621},
{210, 2376786},
{220, 2344951},
{230, 2312102},
{240, 2278261},
{250, 2244898},
{260, 2210815},
{270, 2176413},
{280, 2141673},
{290, 2106648},
{300, 2071291},
{310, 2035838},
{320, 2000134},
{330, 1964313},
{340, 1928283},
{350, 1892304},
{360, 1856135},
{370, 1820179},
{380, 1784019},
{390, 1748069},
{400, 1712162},
{410, 1676426},
{420, 1640999},
{430, 1605474},
{440, 1570325},
{450, 1535495},
{460, 1500905},
{470, 1466667},
{480, 1432675},
{490, 1399265},
{500, 1365844},
{510, 1333223},
{520, 1300773},
{530, 1268832},
{540, 1237500},
{550, 1206600},
{560, 1176215},
{570, 1146132},
{580, 1116723},
{590, 1087762},
{600, 1059318},
{610, 1031461},
{620, 1004263},
{630, 977448},
{640, 951423},
{650, 925536},
{660, 900557},
{670, 876184},
{680, 852083},
{690, 828681},
{700, 805629},
{710, 783368},
{720, 761538},
{730, 740594},
{740, 719730},
{750, 699396},
{760, 679625},
{770, 660449},
{780, 641902},
{790, 624017},
{800, 606356},
{810, 588936},
{820, 572200},
{830, 555936},
{840, 540115},
{850, 524757},
{860, 509779},
{870, 495248},
{880, 481128},
{890, 467434},
{900, 454128},
{910, 441225},
{920, 428629},
{930, 416457},
{940, 404615},
{950, 393165},
{960, 382009},
{970, 371209},
{980, 360719},
{990, 350546},
{1000, 340641},
{1010, 331068},
{1020, 321775},
{1030, 312769},
{1040, 303999},
{1050, 295525},
{1060, 287297},
{1070, 279317},
{1080, 271590},
{1090, 264062},
{1100, 256783},
{1110, 249721},
{1120, 242873},
{1130, 236230},
{1140, 229783},
{1150, 223535},
{1160, 217475},
{1170, 211595},
{1180, 205896},
{1190, 200368},
{1200, 195001},
{1210, 189802},
{1220, 184748},
{1230, 179853},
{1240, 175101},
{1250, 170492},
{1260, 1},
{-333, 0},
};

/*BQ25970: USB_CON_ADC/BAT_Therm_ADC*/
static const struct adc_map_temp adc_map_temp_table2[] = {
{-410, 9999999},
{-400, 49420},
{-390, 49380},
{-380, 49330},
{-370, 49290},
{-360, 49240},
{-350, 49180},
{-340, 49120},
{-330, 49070},
{-320, 49000},
{-310, 48930},
{-300, 48860},
{-290, 48790},
{-280, 48710},
{-270, 48620},
{-260, 48530},
{-250, 48440},
{-240, 48340},
{-230, 48240},
{-220, 48130},
{-210, 48010},
{-200, 47890},
{-190, 47760},
{-180, 47630},
{-170, 47490},
{-160, 47340},
{-150, 47180},
{-140, 47020},
{-130, 46850},
{-120, 46670},
{-110, 46490},
{-100, 46290},
{-90, 46090},
{-80, 45880},
{-70, 45650},
{-60, 45420},
{-50, 45180},
{-40, 44930},
{-30, 44680},
{-20, 44410},
{-10, 44130},
{0, 43840},
{10, 43540},
{20, 43230},
{30, 42910},
{40, 42570},
{50, 42230},
{60, 41880},
{70, 41520},
{80, 41140},
{90, 40760},
{100, 40360},
{110, 39960},
{120, 39540},
{130, 39120},
{140, 38680},
{150, 38240},
{160, 37780},
{170, 37320},
{180, 36850},
{190, 36370},
{200, 35880},
{210, 35380},
{220, 34890},
{230, 34380},
{240, 33850},
{250, 33330},
{260, 32810},
{270, 32270},
{280, 31740},
{290, 31200},
{300, 30650},
{310, 30110},
{320, 29560},
{330, 29010},
{340, 28460},
{350, 27910},
{360, 27360},
{370, 26810},
{380, 26260},
{390, 25710},
{400, 25170},
{410, 24630},
{420, 24090},
{430, 23550},
{440, 23020},
{450, 22500},
{460, 21980},
{470, 21460},
{480, 20950},
{490, 20450},
{500, 19950},
{510, 19460},
{520, 18970},
{530, 18500},
{540, 18030},
{550, 17570},
{560, 17120},
{570, 16670},
{580, 16230},
{590, 15800},
{600, 15380},
{610, 14970},
{620, 14570},
{630, 14170},
{640, 13790},
{650, 13410},
{660, 13040},
{670, 12680},
{680, 12330},
{690, 11980},
{700, 11640},
{710, 11320},
{720, 11000},
{730, 10690},
{740, 10390},
{750, 10090},
{760, 9800},
{770, 9520},
{780, 9250},
{790, 8990},
{800, 8730},
{810, 8480},
{820, 8240},
{830, 8000},
{840, 7770},
{850, 7550},
{860, 7330},
{870, 7120},
{880, 6910},
{890, 6710},
{900, 6520},
{910, 1},
{-333, 0},
};

/*PCA9486: USB_CON_ADC/BAT_Therm_ADC*/
static const struct adc_map_temp adc_map_temp_table2_2[] = {
	{-400, 1630819},
	{-390, 1629437},
	{-380, 1627971},
	{-370, 1626409},
	{-360, 1624756},
	{-350, 1622996},
	{-340, 1621124},
	{-330, 1619148},
	{-320, 1617053},
	{-310, 1614834},
	{-300, 1612483},
	{-290, 1609991},
	{-280, 1607365},
	{-270, 1604596},
	{-260, 1601642},
	{-250, 1598567},
	{-240, 1595292},
	{-230, 1591861},
	{-220, 1588249},
	{-210, 1584420},
	{-200, 1580380},
	{-190, 1576208},
	{-180, 1571727},
	{-170, 1567086},
	{-160, 1562197},
	{-150, 1557074},
	{-140, 1551704},
	{-130, 1546083},
	{-120, 1540191},
	{-110, 1534048},
	{-100, 1527615},
	{-90, 1520893},
	{-80, 1513907},
	{-70, 1506597},
	{-60, 1498985},
	{-50, 1491072},
	{-40, 1482827},
	{-30, 1474282},
	{-20, 1465395},
	{-10, 1456157},
	{0, 1446598},
	{10, 1436712},
	{20, 1426484},
	{30, 1415892},
	{40, 1404975},
	{50, 1393630},
	{60, 1381969},
	{70, 1370055},
	{80, 1357655},
	{90, 1345010},
	{100, 1331959},
	{110, 1318542},
	{120, 1304812},
	{130, 1290836},
	{140, 1276528},
	{150, 1261765},
	{160, 1246775},
	{170, 1231644},
	{180, 1216018},
	{190, 1200164},
	{200, 1183899},
	{210, 1167544},
	{220, 1151210},
	{230, 1134375},
	{240, 1117055},
	{250, 1100000},
	{260, 1082600},
	{270, 1065060},
	{280, 1047371},
	{290, 1029560},
	{300, 1011604},
	{310, 993624},
	{320, 975540},
	{330, 957422},
	{340, 939223},
	{350, 921073},
	{360, 902853},
	{370, 884765},
	{380, 866599},
	{390, 848563},
	{400, 830573},
	{410, 812692},
	{420, 794990},
	{430, 777262},
	{440, 759745},
	{450, 742410},
	{460, 725216},
	{470, 708220},
	{480, 691367},
	{490, 674823},
	{500, 658295},
	{510, 642182},
	{520, 626173},
	{530, 610434},
	{540, 595013},
	{550, 579823},
	{560, 564903},
	{570, 550147},
	{580, 535738},
	{590, 521564},
	{600, 507658},
	{610, 494053},
	{620, 480783},
	{630, 467713},
	{640, 455041},
	{650, 442448},
	{660, 430308},
	{670, 418473},
	{680, 406782},
	{690, 395438},
	{700, 384275},
	{710, 373504},
	{720, 362949},
	{730, 352831},
	{740, 342759},
	{750, 332951},
	{760, 323421},
	{770, 314184},
	{780, 305257},
	{790, 296654},
	{800, 288165},
	{810, 279796},
	{820, 271762},
	{830, 263958},
	{840, 256372},
	{850, 249012},
	{860, 241838},
	{870, 234882},
	{880, 228126},
	{890, 221577},
	{900, 215218},
	{910,	1},
	{-333,	0},
};

/*LCM /BQ adc*/
static const struct adc_map_temp adc_map_temp_table3[] = {
{-300, 9999999},
{-29, 1430433},
{-28, 1428647},
{-27, 1426761},
{-26, 1424748},
{-25, 1422650},
{-24, 1420414},
{-23, 1418069},
{-22, 1415598},
{-21, 1412976},
{-20, 1410206},
{-19, 1407341},
{-18, 1404261},
{-17, 1401067},
{-16, 1397697},
{-15, 1394161},
{-14, 1390448},
{-13, 1386556},
{-12, 1382469},
{-11, 1378201},
{-10, 1373722},
{-9, 1369033},
{-8, 1364151},
{-7, 1359031},
{-6, 1353688},
{-5, 1348121},
{-4, 1342307},
{-3, 1336265},
{-2, 1329966},
{-1, 1323400},
{0, 1316587},
{1, 1309521},
{2, 1302188},
{3, 1294570},
{4, 1286694},
{5, 1278481},
{6, 1270010},
{7, 1261324},
{8, 1252251},
{9, 1242964},
{10, 1233341},
{11, 1223408},
{12, 1213202},
{13, 1202769},
{14, 1192041},
{15, 1180924},
{16, 1169583},
{17, 1158082},
{18, 1146150},
{19, 1133984},
{20, 1121440},
{21, 1108764},
{22, 1096038},
{23, 1082855},
{24, 1069218},
{25, 1055718},
{26, 1041870},
{27, 1027833},
{28, 1013598},
{29, 999184},
{30, 984570},
{31, 969852},
{32, 954963},
{33, 939959},
{34, 924800},
{35, 909593},
{36, 894237},
{37, 878903},
{38, 863410},
{39, 847938},
{40, 832413},
{41, 816892},
{42, 801435},
{43, 785866},
{44, 770391},
{45, 754988},
{46, 739623},
{47, 724346},
{48, 709113},
{49, 694075},
{50, 678966},
{51, 664157},
{52, 649362},
{53, 634738},
{54, 620333},
{55, 606069},
{56, 591987},
{57, 577989},
{58, 564250},
{59, 550669},
{60, 537280},
{61, 524117},
{62, 511220},
{63, 498458},
{64, 486028},
{65, 473622},
{66, 461610},
{67, 449850},
{68, 438184},
{69, 426820},
{70, 415591},
{71, 404715},
{72, 394017},
{73, 383723},
{74, 373440},
{75, 363390},
{76, 353592},
{77, 344063},
{78, 334823},
{79, 325891},
{80, 317050},
{81, 308307},
{82, 299889},
{83, 291690},
{84, 283697},
{85, 275920},
{86, 268321},
{87, 260933},
{88, 253740},
{89, 246750},
{90, 239947},
{91, 233336},
{92, 226872},
{93, 220616},
{94, 214518},
{95, 208613},
{96, 202850},
{97, 197263},
{98, 191828},
{99, 186550},
{100, 181404},
{101, 176423},
{102, 171582},
{103, 166884},
{104, 162304},
{105, 157873},
{106, 153565},
{107, 149383},
{108, 145329},
{109, 141375},
{110, 137548},
{111, 133831},
{112, 130224},
{113, 126721},
{114, 123318},
{115, 120018},
{116, 116814},
{117, 113702},
{118, 110684},
{119, 107754},
{120, 104907},
{-333, 0},
};
/*adapter/adapter_conn adc*/
static const struct adc_map_temp adc_map_temp_table4[] = {
	{-333, 0},
	{-300, 44138},
	{-290, 47013},
	{-280, 50053},
	{-270, 53269},
	{-260, 56669},
	{-250, 60262},
	{-240, 64058},
	{-230, 68065},
	{-220, 72294},
	{-210, 76753},
	{-200, 81452},
	{-190, 86400},
	{-180, 91607},
	{-170, 97083},
	{-160, 102836},
	{-150, 108877},
	{-140, 115215},
	{-130, 121861},
	{-120, 128823},
	{-110, 136113},
	{-100, 143741},
	{-90, 151716},
	{-80, 160050},
	{-70, 168754},
	{-60, 177838},
	{-50, 187313},
	{-40, 197191},
	{-30, 207483},
	{-20, 218201},
	{-10, 229355},
	{0, 240957},
	{10, 253019},
	{20, 265552},
	{30, 278566},
	{40, 292073},
	{50, 306082},
	{60, 320604},
	{70, 335648},
	{80, 351223},
	{90, 367337},
	{100, 383998},
	{110, 401212},
	{120, 418985},
	{130, 437323},
	{140, 456228},
	{150, 475704},
	{160, 495752},
	{170, 516372},
	{180, 537565},
	{190, 559327},
	{200, 581655},
	{210, 604544},
	{220, 627989},
	{230, 651982},
	{240, 676514},
	{250, 701575},
	{260, 727153},
	{270, 753236},
	{280, 779810},
	{290, 806859},
	{300, 834368},
	{310, 862318},
	{320, 890692},
	{330, 919470},
	{340, 948632},
	{350, 978156},
	{360, 1008021},
	{370, 1038203},
	{380, 1068681},
	{390, 1099430},
	{400, 1130425},
	{410, 1161643},
	{420, 1193059},
	{430, 1224648},
	{440, 1256384},
	{450, 1288242},
	{460, 1320198},
	{470, 1352226},
	{480, 1384302},
	{490, 1416400},
	{500, 1448496},
	{510, 1480566},
	{520, 1512587},
	{530, 1544535},
	{540, 1576387},
	{550, 1608121},
	{560, 1639715},
	{570, 1671148},
	{580, 1702399},
	{590, 1733450},
	{600, 1764280},
	{610, 1794872},
	{620, 1825208},
	{630, 1855270},
	{640, 1885043},
	{650, 1914512},
	{660, 1943663},
	{670, 1972481},
	{680, 2000954},
	{690, 2029070},
	{700, 2056818},
	{710, 2084189},
	{720, 2111172},
	{730, 2137759},
	{740, 2163943},
	{750, 2189716},
	{760, 2215073},
	{770, 2240009},
	{780, 2264518},
	{790, 2288598},
	{800, 2312245},
	{810, 2335458},
	{820, 2358234},
	{830, 2380573},
	{840, 2402474},
	{850, 2423939},
	{860, 2444967},
	{870, 2465561},
	{880, 2485722},
	{890, 2505454},
	{900, 2524759},
	{910, 2543641},
	{920, 2562103},
	{930, 2580150},
	{940, 2597787},
	{950, 2615019},
	{960, 2631850},
	{970, 2648287},
	{980, 2664335},
	{990, 2680001},
	{1000, 2695290},
	{1010, 2710209},
	{1020, 2724764},
	{1030, 2738963},
	{1040, 2752811},
	{1050, 2766315},
	{1060, 2779483},
	{1070, 2792322},
	{1080, 2804838},
	{1090, 2817038},
	{1100, 2828929},
	{1110, 2840518},
	{1120, 2851813},
	{1130, 2862819},
	{1140, 2873544},
	{1150, 2883994},
	{1160, 2894176},
	{1170, 2904096},
	{1180, 2913762},
	{1190, 2923178},
	{1200, 2932353},
	{1210, 2941290},
	{1220, 2949998},
	{1230, 2958481},
	{1240, 2966745},
	{1250, 2974796},
	{1260, 9999999},
};

