import math

# .. prototype 1 .. to test hidding from a immobile seeker, one obstacle, obstacle and hidder are not too close together
# first find where is seeker (seeker will not move)
# record d1
# then turn 360 to the right
# record d2 and a1 if found obstacle
# record a1 when the obstalce is at the center of the screen ?
# choose the closest obsatcle
# input a1,d1,d2 of that obstacle into case chooser
# casechooser return which case to choose and execute the case
# robot is hidden

# a1 = angle SHO in degree, S = seeker, H = hidder, O = obstacle
# a2 = angle SOH
# a3 = angle HSO
# a4,5,6 = utility angles
# d1 = distance S to H in cm
# d2 = distance H to O
# d3 = distance S to O

oblength = 45


def findSandOb():
    print ("Start finding objectives")
    print ("turning 360")
    a1 = 0
    d1 = 0
    d2 = 0
    return a1, d1, d2


def caseChooser(a1, d1, d2, oblength):
    if (a1 > 5 and a1 < 80):
        a3, d3 = computeA3(a1, d1, d2)
        # case4/5/6 right
        print()
        if a3 >= 90 and a3 < 180:
            # case5 right
            case5(a3, d1, d3, oblength, "right")
            print ()
        else:
            # case4/6
            a5, k, _ = computeA5(a3, d1, d3, oblength)
            if a5 < 90:
                # case6 right
                case6(a5, k, "right")
                print ()
            else:
                # case4 right
                case4(a3, d1, "right")
                print ()

    elif a1 > 280 and a1 < 355:
        a1 = 360 - a1
        a3, d3 = computeA3(a1, d1, d2)
        # case4/5/6 left
        print()
        if a3 >= 90 and a3 < 180:
            # case5 left
            case5(a3, d1, d3, oblength, "left")
            print ()
        else:
            # case4/6
            a5, k, _ = computeA5(a3, d1, d3, oblength)
            if a5 < 90:
                # case6 left
                case6(a5, k, "left")
                print ()
            else:
                # case4 left
                case4(a3, d1, "left")
                print ()

    elif a1 < 5 or a1 > 355:
        # case7
        case7(d1, oblength)
        print ()
    elif a1 > 80 and a1 < 175:
        # case9 right
        a3, d3 = computeA3(a1, d1, d2)
        case9(a3, d1, d3, oblength, "right")
        print()
    elif a1 > 185 and a1 < 280:
        # case9 left
        a1 = 360 - a1
        a3, d3 = computeA3(a1, d1, d2)
        case9(a3, d1, d3, oblength, "left")
        print()
    elif a1 > 175 and a1 < 185:
        # case 10
        case10(d2, oblength)
        print()
    else:
        print ("no case found")


def computeA3(a1, d1, d2):
    d3 = d2 * d2 + d1 * d1 - 2 * d1 * d2 * math.cos(math.radians(a1))
    d3 = math.sqrt(d3)
    print ("d3:", d3)
    # a3 = d2 * math.sin(math.radians(a1)) / d3
    # a3 = math.degrees(math.asin(a3))
    # print ("a3:",a3)
    aa = (d2 * d2 - d1 * d1 - d3 * d3) / (- 2 * d1 * d3)
    a3 = math.degrees(math.acos(aa))
    print ("A3:", a3)
    return a3, d3


def computeA5(a3, d1, d3, oblength):
    h = d1 * math.sin(math.radians(a3))
    a4 = math.degrees(math.atan(h / (d3 + oblength)))
    a5 = 180 - a3 - a4
    # a5 is the degree to move
    k = h / (math.sin(math.radians(a4)))
    return a5, k, a4


def case4(a3, d1, direction):
    print ("Starting case 4 ... ")
    x = d1 * math.tan(math.radians(a3))
    if direction == "right":
        print ("x:", x)
        print ("Start by facing seeker, turn 90 degree to right")
        print ("move forward to a distance of", x, "cm")
        print ("turn left 90 +", a3, "degree")
        print ("move near to the obstacle")
    elif direction == "left":
        print ("x:", x)
        print ("Start by facing seeker, turn 90 degree to left")
        print ("move forward to a distance of", x, "cm")
        print ("turn right 90 + ", a3, "degree")
        print ("move near to the obstacle")
    print ("Case 4 ends ... \n")
    print ("")


def case6(a5, k, direction):
    print ("Starting case 6 ...")
    if direction == "right":
        print ("Start by facing seeker")
        print ("turn right a5:", a5)
        print ("move forward k:", k)
        print ("turn left 180 - a5:", (180 - a5))
    elif direction == "left":
        print ("Start by facing seeker")
        print ("turn left a5:", a5)
        print ("move forward k:", k)
        print ("turn right 180 - a5:", (180 - a5))
    print ("move near obstacle")
    print ("Case 6 ends ... \n")


def case5(a3, d1, d3, oblength, direction):
    print ("Starting case 5 ...")
    if direction == "right":
        a4 = 180 - a3
        print ("Start by facing seeker, turn", a4, "degree to right")
        print ("move forward d3:", d3, "+ oblength:", oblength)
        print ("turn", a4, "to the left")
        print ("move forward d1:", d1)
        print ("turn left", a3, "degree")
    elif direction == "left":
        a4 = 180 - a3
        print ("Start by facing seeker, turn", a4, "degree to left")
        print ("move forward d3:", d3, "+ oblength:", oblength)
        print ("turn", a4, "to the right")
        print ("move forward d1:", d1)
        print ("turn right", a3, "degree")
    print ("move near to the obstacle")
    print ("Case 5 ends ... \n")


def case7(d1, oblength):
    print ("Start case 7 ...")
    print ("turn right 90 degree")
    print ("move forward", oblength, "cm")
    print ("turn left 90 degree")
    print ("move forward", d1 * 1.5, "cm")
    # search for seeker and obstacle again
    a1, d1, d2 = findSandOb()
    # caseChooser(a1, d1, d2, oblength)
    print ("Case 7 ends ... \n")


def case9(a3, d1, d3, oblength, direction):
    print ("Case 9 starts ...")
    a5, k, a4 = computeA5(a3, d1, d3, oblength)
    a6 = 180 - a4
    if direction == "right":
        print ("Start by facing seeker")
        print ("turn right", a6, "degree")
        print ("move forward", k, "cm")
        print ("move left", a6)
    elif direction == "left":
        print ("Start by facing seeker")
        print ("turn left", a6, "degree")
        print ("move forward", k, "cm")
        print ("move right", a6)
    print ("move near to obstacle")
    print ("Case 9 ends ... \n")


def case10(d2, oblength):
    print ("Case 10 starts ... ")
    a4 = math.degrees(math.atan(oblength / (d2 + oblength)))
    k = oblength / (math.sin(math.radians(a4)))
    a5 = 90 - a4
    print ("Start by facing seeker")
    aStart = 180 - a4
    print ("turn right", aStart, "degree")
    print ("move forward", k, "cm")
    aNext = 180 - a5
    print ("turn right", aNext, "degree")
    print ("move forwad", oblength, "cm")
    print ("turn right", 90, "degree")
    print ("move near obstacle")
    print ("case 10 ends ..\n")


caseChooser(180, 100, 300, 50)

case5(55, 100, 200, 45, "left")
case6(55, 100, "left")
case7(100, 50)
case9(125, 100, 200, 50, "left")

caseChooser(180, 100, 300, 50)
