#!/usr/bin/python
# -*- coding: utf-8 -*-

tagMap = {
  128: [135, 340.0, 0, 0.0],
  1: [2, 587.0, 111, 487.0],
  2: [3, 217.0, 11, 881.0],
  3: [4, 145.0, 112, 375.0],
  4: [101, 300.0, 6, 376.0],
  5: [103, 340.0, 106, 315.0],
  6: [13, 447.0, 117, 253.0],
  7: [108, 341.0, 114, 222.0],
  8: [113, 265.0, 7, 261.0],
  9: [0, 0.0, 139, 985.0],
  10: [0, 0.0, 131, 459.0],
  11: [129, 410.0, 126, 260.0],
  12: [127, 262.0, 128, 275.0],
  13: [136, 605.0, 0, 0.0],
  14: [121, 197.0, 15, 357.0],
  15: [124, 309.0, 123, 247.0],
  16: [8, 360.0, 14, 180.0],
  17: [16, 233.0, 125, 487.0],
  18: [134, 285.0, 17, 290.0, 132, 245.0],
  19: [137, 302.0, 141, 220.0],
  20: [138, 295.0, 133, 595.0],
  132: [0, 0.0, 0, 0.0, 143, 1103.0],
  133: [0, 0.0, 122, 498.0],
  134: [20, 540.0, 0, 0.0],
  129: [0, 0.0, 135, 288.0],
  135: [140, 267.0, 0, 0.0],
  136: [19, 462.0, 0, 0.0],
  137: [0, 0.0, 0, 0.0, 142, 1225.0],
  138: [18, 550.0, 0, 0.0],
  139: [0, 0.0, 140, 337.0],
  130: [0, 0.0, 18, 472.0],
  140: [0, 0.0, 19, 340.0],
  141: [0, 0.0, 20, 460.0],
  142: [105, 153.0, 0, 0.0],
  143: [0, 0.0, 105, 157.0],
  131: [139, 615.0, 0, 0.0],
  101: [102, 300.0, 0, 0.0],
  102: [5, 280.0, 0, 0.0],
  103: [104, 225.0, 0, 0.0],
  104: [1, 1910.0, 0, 0.0],
  105: [0, 0.0, 102, 163.0],
  106: [0, 0.0, 0, 0.0, 109, 163.0],
  107: [0, 0.0, 104, 202.0],
  108: [0, 0.0, 0, 0.0, 107, 163.0],
  109: [116, 207.0, 0, 0.0],
  110: [0, 0.0, 1, 1998.0],
  111: [10, 600.0, 9, 520.0],
  112: [0, 0.0, 12, 437.0],
  113: [0, 0.0, 5, 415.0],
  114: [115, 155.0, 0, 0.0],
  115: [0, 0.0, 116, 220.0],
  116: [110, 207.0, 0, 0.0],
  117: [12, 273.0, 0, 0.0],
  118: [0, 0.0, 110, 310.0],
  119: [0, 0.0, 115, 352.0],
  120: [118, 374.0, 0, 0.0],
  121: [119, 203.0, 120, 298.0],
  122: [0, 0.0, 118, 320.0],
  123: [130, 190.0, 0, 0.0],
  124: [122, 183.0, 0, 0.0],
  125: [0, 0.0, 130, 420.0],
  126: [131, 288.0, 0, 0.0],
  127: [0, 0.0, 136, 433.0],
}

tagPoses = {
  128: [575.0, 619.0],
  1: [389.0, 1720.0, 3.067867687146804],
  2: [205.0, 1494.0, -0.49110483132568117],
  3: [348.0, 1570.0, 0.08047101126546087],
  4: [490.0, 1574.0, 0.01538340178059515],
  5: [1373.0, 1574.0, 0.016127633843636236],
  6: [706.0, 1398.0, -1.292421800240984],
  7: [1375.0, 1215.0, 1.0605780727780427],
  8: [1324.0, 999.0, 1.555645970920127],
  9: [124.0, 949.0, -1.5551725981744198],
  10: [275.0, 955.0, -1.5813475914516975],
  11: [425.0, 958.0, -1.5813830196116756],
  12: [575.0, 960.0, -1.5707963267948966],
  13: [725.0, 953.0, -1.5707963267948966],
  14: [1380.0, 772.0, 0.9484742156379248],
  15: [1711.0, 822.0, 0.046840712915969654],
  16: [1327.0, 639.0, 1.6010900867136717],
  17: [1368.0, 433.0, 2.406386934184464],
  18: [1613.0, 373.0, 3.1098572800613353],
  19: [947.0, 75.0, 0.0],
  20: [1627.0, 75.0, 0.0],
  132: [1306.0, 375.0],
  133: [2075.0, 319.0],
  134: [1325.0, 281.0],
  129: [500.0, 526.0],
  135: [575.0, 281.0],
  136: [725.0, 284.0],
  137: [1175.0, 244.0],
  138: [1925.0, 169.0],
  139: [331.0, 75.0],
  130: [1925.0, 506.0],
  140: [669.0, 75.0],
  141: [1231.0, 75.0],
  142: [920.0, 1400.0],
  143: [1130.0, 1400.0],
  131: [275.0, 431.0],
  101: [856.0, 1575.0],
  102: [1156.0, 1575.0],
  103: [1776.0, 1575.0],
  104: [2000.0, 1575.0],
  105: [1025.0, 1510.0],
  106: [1722.0, 1490.0],
  107: [1825.0, 1489.0],
  108: [1723.0, 1361.0],
  109: [1824.0, 1362.0],
  110: [2075.0, 1446.0],
  111: [125.0, 1406.0],
  112: [575.0, 1331.0],
  113: [1325.0, 1331.0],
  114: [1628.0, 1275.0],
  115: [1786.0, 1275.0],
  116: [2006.0, 1275.0],
  117: [650.0, 1122.0],
  118: [2075.0, 1136.0],
  119: [1624.0, 1050.0],
  120: [1831.0, 975.0],
  121: [1606.0, 852.0],
  122: [2075.0, 817.0],
  123: [1925.0, 694.0],
  124: [2000.0, 674.0],
  125: [1644.0, 675.0],
  126: [350.0, 674.0],
  127: [650.0, 675.0],
}

tagDets = {
  128: [575.0, 620.0],
  1: [324.18106494028405, 1724.787450813407],
  2: [263.48449233979875, 1462.7220172526086],
  3: [410.0, 1575.0],
  4: [555.0, 1575.0],
  5: [1435.0, 1575.0],
  6: [723.5396561121818, 1336.6285983008972],
  7: [1408.3317624972335, 1274.558729336779],
  8: [1325.0, 1065.0],
  9: [125.0, 885.0],
  10: [274.305733707744, 889.2030978643313],
  11: [424.3159598858941, 893.3892126223341],
  12: [575.0, 895.0],
  13: [725.0, 890.0],
  14: [1417.5690094338888, 824.3667949710759],
  15: [1775.0, 825.0],
  16: [1325.0, 705.0],
  17: [1324.553720552072, 472.29005004044234],
  18: [1550.0, 375.0],
  19: [1010.0, 75.0],
  20: [1690.0, 75.0],
  132: [1305.0, 375.0],
  133: [2073.418271264227, 317.266982662052],
  134: [1325.6332050289238, 282.4309905661091],
  129: [499.25204338346776, 525.2247434088978],
  135: [575.0, 280.0],
  136: [725.0, 285.0],
  137: [1173.917261699878, 245.1417609622411],
  138: [1924.3667949710762, 167.56900943388865],
  139: [332.92848749087983, 76.78227906953519],
  130: [1925.0, 505.0],
  140: [670.0, 75.0],
  141: [1230.0, 75.0],
  142: [919.9303128406068, 1402.5884800001807],
  143: [1132.9654774784221, 1399.3977090089197],
  131: [275.0, 430.0],
  101: [855.0, 1575.0],
  102: [1155.0, 1575.0],
  103: [1775.0, 1575.0],
  104: [2000.0, 1575.0],
  105: [1026.5347440018577, 1509.6525205544933],
  106: [1723.3753110441664, 1489.2739954750464],
  107: [1824.8277363085951, 1488.0394744862288],
  108: [1723.3753110441648, 1360.7260045249523],
  109: [1824.8277363085956, 1361.9605255137717],
  110: [2075.0, 1445.0],
  111: [125.0, 1405.0],
  112: [573.5202283145229, 1332.3452359070666],
  113: [1325.0, 1330.0],
  114: [1630.0, 1275.0],
  115: [1785.0, 1275.0],
  116: [2004.9384417029758, 1275.0],
  117: [650.7479511578027, 1125.2247355026796],
  118: [2075.0, 1135.0],
  119: [1625.0, 1050.0],
  120: [1828.5851415547468, 973.8168933869216],
  121: [1606.5638307124702, 852.1367529552825],
  122: [2075.0, 815.0],
  123: [1925.0, 695.0],
  124: [1999.2520488421974, 675.2247355026796],
  125: [1642.4469209021452, 673.438504065985],
  126: [350.74795115780256, 675.2247355026793],
  127: [649.2520488421974, 675.2247355026793],
}

def findClosestNode(p, nodes=tagDets):
  opt = (None, 1000**2)
  for k, v in nodes.iteritems():
    d = (v[0] - p[0])**2 + (v[1] - p[1])**2
    if d < opt[1]:
      opt = (k, d)
  if opt[0] is not None:
    return (opt[0], nodes[opt[0]])
  else:
    return None
