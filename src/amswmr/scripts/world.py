#!/usr/bin/python
# -*- coding: utf-8 -*-

BOX = (0, 0, 2200, 1800)

EDGES = {
  '16-17': [('A', 650, 1200, 75, 0, -90),
            ('A', 650, 1050, 75, 90, 90)],
  '24-30': [('L', 425, 750, 425, 600),
            ('A', 500, 600, 75, -180, 90),
            ('A', 500, 450, 75, 90, -90)],
  '35-36': [('L', 800, 75, 1100, 75)],
  '12-8':  [('B', 1550, 1275, 1775, 1275, 1775, 1575, 2000, 1575)],
  '25-27': [('A', 650, 750, 75, -180, 90),
            ('A', 650, 600, 75, 90, -90)],
  '26-33': [('L', 275, 600, 275, 300),
            ('A', 200, 300, 75, 0, -90),
            ('A', 200, 150, 75, 90, 90)],
  '10-33': [('L', 125, 1350, 125, 150)],
  '41-5':  [('A', 1100, 1500, 75, 180, -90)],
  '21-22': [('L', 1550, 825, 1850, 825),
            ('A', 1850, 750, 75, 90, -90)],
  '4-16':  [('A', 650, 1500, 75, 90, -90),
            ('L', 725, 1500, 725, 1200)],
  '21-18': [('A', 1550, 900, 75, -90, 90)],
  '34-35': [('L', 650, 75, 800, 75)],
  '31-28': [('A', 1400, 450, 75, -90, -90),
            ('L', 1325, 450, 1325, 600)],
  '33-34': [('A', 200, 150, 75, -180, 90),
            ('L', 200, 75, 650, 75)],
  '17-25': [('L', 575, 1050, 575, 750)],
  '15-1':  [('L', 2075, 1350, 2075, 1650)],
  '9-2':   [('A', 200, 1500, 75, -180, 90),
            ('A', 200, 1500, 75, -90, 90),
            ('A', 350, 1500, 75, 180, -90)],
  '13-14': [('L', 1700, 1275, 2000, 1275)],
  '3-17':  [('A', 500, 1500, 75, 90, -90),
            ('L', 575, 1500, 575, 1050)],
  '36-40': [('A', 1100, 150, 75, -90, 90),
            ('L', 1175, 150, 1175, 300)],
  '2-24':  [('A', 350, 1500, 75, 90, -90),
            ('L', 425, 1500, 425, 750)],
  '6-7':   [('L', 1325, 1575, 1550, 1575)],
  '27-35': [('L', 725, 600, 725, 150),
            ('A', 800, 150, 75, -180, 90)],
  '38-23': [('L', 1850, 75, 2000, 75),
            ('A', 2000, 150, 75, -90, 90),
            ('L', 2075, 150, 2075, 750)],
  '22-29': [('L', 1925, 750, 1925, 600)],
  '1-9':   [('A', 2000, 1650, 75, 0, 90),
            ('L', 2000, 1725, 200, 1725), 
            ('A', 200, 1650, 75, 90, 90),
            ('L', 125, 1650, 125, 1500)],
  '2-3':   [('L', 350, 1575, 500, 1575)],
  '18-19': [('A', 1700, 900, 75, 180, -90),
            ('L', 1700, 975, 2000, 975),
            ('A', 2000, 1050, 75, -90, 90)],
  '36-37': [('L', 1100, 75, 1400, 75)],
  '11-12': [('A', 1400, 1200, 75, 180, -90),
            ('L', 1400, 1275, 1550, 1275)],
  '25-30': [('L', 575, 750, 575, 450)],
  '10-26': [('A', 200, 1350, 75, -180, 90),
            ('A', 200, 1200, 75, 90, -90),
            ('L', 275, 1200, 275, 600)],
  '24-26': [('A', 350, 750, 75, 0, -90),
            ('A', 350, 600, 75, 90, 90)],
  '7-14':  [('B', 1550, 1575, 1775, 1575, 1775, 1275, 2000, 1275)],
  '29-32': [('L', 1925, 600, 1925, 450),
            ('A', 1850, 450, 75, 0, -90)],
  '32-31': [('L', 1850, 375, 1400, 375)],
  '5-6':   [('L', 1100, 1575, 1325, 1575)],
  '31-37': [('A', 1400, 300, 75, 90, 90),
            ('L', 1325, 300, 1325, 150),
            ('A', 1400, 150, 75, -180, 90)],
  '20-21': [('A', 1400, 750, 75, 180, -90),
            ('L', 1400, 825, 1550, 825)],
  '31-39': [('L', 1400, 375, 1250, 375)],
  '23-19': [('L', 2075, 750, 2075, 1050)],
  '14-15': [('A', 2000, 1350, 75, -90, 90)],
  '9-10':  [('L', 125, 1500, 125, 1350)],
  '22-23': [('A', 2000, 750, 75, -180, 90),
            ('A', 2000, 750, 75, -90, 90)],
  '28-20': [('L', 1325, 600, 1325, 750)],
  '30-34': [('L', 575, 450, 575, 150),
            ('A', 650, 150, 75, -180, 90)],
  '7-8':   [('L', 1550, 1575, 2000, 1575)],
  '4-5':   [('L', 650, 1575, 1100, 1575)],
  '12-13': [('L', 1550, 1275, 1700, 1275)],
  '19-15': [('L', 2075, 1050, 2075, 1350)],
  '28-29': [('A', 1400, 600, 75, 180, -90),
            ('L', 1400, 675, 1850, 675),
            ('A', 1850, 600, 75, 90, -90)],
  '3-4':   [('L', 500, 1575, 650, 1575)],
  '16-27': [('L', 725, 1200, 725, 600)],
  '37-38': [('L', 1400, 75, 1850, 75)],
  '20-11': [('L', 1325, 750, 1325, 1200)],
  '11-6':  [('L', 1325, 1200, 1325, 1350),
            ('L', 1325, 1350, 1250, 1500),
            ('A', 1325, 1500, 75, 180, -90)],
  '18-13': [('L', 1625, 900, 1625, 1200),
            ('A', 1700, 1200, 75, 180, -90)],
  '38-32': [('A', 1850, 150, 75, -90, 90),
            ('L', 1925, 150, 1925, 300),
            ('A', 1850, 300, 75, 0, 90)],
  '8-1':   [('A', 2000, 1650, 75, -90, 90)],
  '39-41': [('X', 1250, 375, 1181, 1356),
            ('L', 1181, 1356, 1025, 1500)],
  '40-41': [('X', 1175, 300, 867, 1356),
            ('L', 867, 1356, 1025, 1500)],
}

TAGS = {
#  # Real tags
#    1: ( 275, 1725),
#    2: ( 200, 1500),
#    3: ( 413, 1575),
#    4: ( 552, 1575),
#    5: (1475, 1567),
#    6: ( 725, 1305),
#    7: (1388, 1224),
#    8: (1325, 1125),
#    9: ( 125,  847),
#   10: ( 275,  849),
#   11: ( 425,  855),
#   12: ( 575,  860),
#   13: ( 725,  852),
#   14: (1475,  799),
#   15: (1775,  825),
#   16: (1338,  675),
#   17: (1377,  463),
#   18: (1504,  375),
#   19: (1010,   75),
#   20: (1752,   75),
   # Real tags
    1: (389, 1720),
    2: (205, 1494),
    3: (348, 1570),
    4: (490, 1574),
    5: (1373, 1574),
    6: (706, 1398),
    7: (1375, 1215),
    8: (1324, 999),
    9: (124, 949),
   10: (275, 955),
   11: (425, 958),
   12: (575, 960),
   13: (725, 953),
   14: (1380, 772),
   15: (1711, 822),
   16: (1327, 639),
#   16: (1333, 576), # Alternative tag
   17: (1368, 433),
   18: (1613, 373),
   19: (947, 75),
#   19: (856, 85), # Alternative tag
   20: (1627, 75),
#   20: (1486, 78), # Alternative tag
  # Virtual tags
  101: ( 856, 1575),
  102: (1156, 1575),
  103: (1776, 1575),
  104: (2000, 1575),
  105: (1025, 1510),
  106: (1722, 1490),
  107: (1825, 1489),
  108: (1723, 1361),
  109: (1824, 1362),
  110: (2075, 1446),
  111: ( 125, 1406),
  112: ( 575, 1331),
  113: (1325, 1331),
  114: (1628, 1275),
  115: (1786, 1275),
  116: (2006, 1275),
  117: ( 650, 1122),
  118: (2075, 1136),
  119: (1624, 1050),
  120: (1831,  975),
  121: (1606,  852),
  122: (2075,  817),
  123: (1925,  694),
  124: (2000,  674),
  125: (1644,  675),
  126: ( 350,  674),
  127: ( 650,  675),
  128: ( 575,  619),
  129: ( 500,  526),
  130: (1925,  506),
  131: ( 275,  431),
  132: (1306,  375),
  133: (2075,  319),
  134: (1325,  281),
  135: ( 575,  281),
  136: ( 725,  284),
  137: (1175,  244),
  138: (1925,  169),
  139: ( 331,   75),
  140: ( 669,   75),
  141: (1231,   75),
  142: (920, 1400),
  143: (1130, 1400),
}

NODES = {
   1: (2075, 1650),
   2: ( 350, 1575),
   3: ( 500, 1575),
   4: ( 650, 1575),
   5: (1100, 1575),
   6: (1325, 1575),
   7: (1550, 1575),
   8: (2000, 1575),
   9: ( 125, 1500),
  10: ( 125, 1350),
  11: (1325, 1200),
  12: (1550, 1275),
  13: (1700, 1275),
  14: (2000, 1275),
  15: (2075, 1350),
  16: ( 725, 1200),
  17: ( 575, 1050),
  18: (1625,  900),
  19: (2075, 1050),
  20: (1325,  750),
  21: (1550,  825),
  22: (1925,  750),
  23: (2075,  750),
  24: ( 425,  750),
  25: ( 575,  750),
  26: ( 275,  600),
  27: ( 725,  600),
  28: (1325,  600),
  29: (1925,  600),
  30: ( 575,  450),
  31: (1400,  375),
  32: (1850,  375),
  33: ( 125,  150),
  34: ( 650,   75),
  35: ( 800,   75),
  36: (1100,   75),
  37: (1400,   75),
  38: (1850,   75),
  39: (1250,  375),
  40: (1175,  300),
  41: (1025, 1500),
}

MTAG = {
  584190835274: 1,
  584191425717: 2,
  584190573134: 3,
  584191294388: 4,
  584190048838: 5,
  584190310978: 6,
  584191556528: 7,
  584189523795: 8,
  584184871702: 9,
  584184609578: 10,
  584184347438: 11,
  584184085282: 12,
  584183823142: 13,
  584189785943: 14,
  584189326943: 15,
  584183561018: 16, #584185199377 # Alternative tag
  584182905652: 17,
  584199420872: 18,
  584194635661: 19, #584189786714 # Alternative tag
  584191687865: 20 #584194307978 (not detected) or 584191687865 (alternative tag)
}

SPACES = [((797,       225+1135),
           (797,       225),
           (797+454,   225),
           (797+454,   225+1135),
           (797+454/2, 225+1135+216))]
