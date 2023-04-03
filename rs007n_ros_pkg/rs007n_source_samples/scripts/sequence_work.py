#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import rs007n_commander

# 教師位置で得た収納レーン位置　①
PosisionsData = {
    "Home": [0.0, 7.669904334761668e-06, -1.5707907676696777, 0.0, -1.5707603693008423, -6.447359919548035e-05],
    
    "TPReturnSet": [-1.0790308713912964, 0.20854470133781433, -2.0828843116760254, 0.5306193232536316, -1.1917473077774048, 0.7924346327781677],
    
    "TPReturnPickTop": [-0.48567622155941326, 0.9776145025131351, -1.2890714456420689, 0.253421167940874, -1.3827067993331221, 0.3496369836176152],
    "TPReturnPick": [-0.48563915491104126, 1.076501727104187, -1.2622729539871216, 0.2573099434375763, -1.3140223026275635, 0.33081114292144775],
    
    "TPColorSetTop": [-0.4855752545914678,0.9782795554929331,-1.2882967018296112,0.2528624413521783,-1.3845345684921568, 0.3502947261554299],
    "TPColorSet": [1.098253607749939, 0.5768535137176514, -2.0086557865142822, -0.002377670258283615, -0.5553370118141174, -1.096225619316101],
    
    "TPBoxRedSetTop": [0.8718475945994744,0.6014063683917996,-1.7375523105823127,0.5210077553778376,-0.3612087420135328,-0.7390130128192478],
    "TPBoxRedSet":  [0.8721544146537781, 0.7063138484954834, -1.740060806274414, 0.7102177739143372, -0.27383953332901, -0.9376193881034851],
    "TPBoxRedPick": [0.9885355830192566, 0.7154333591461182, -2.0532805919647217, 1.9392739534378052, -0.24723456799983978, -2.3016209602355957],
    
    "TPBoxBlueSetTop": [0.46040565696363345, 0.6766691145436967, -1.6049057613410236, 0.29468531249138774, -0.3712603706040376, -0.6880283056466254],
    "TPBoxBlueSet": [0.4605681598186493, 0.7740544080734253, -1.607384204864502, 0.3973470628261566, -0.2777823507785797, -0.7954598665237427],
    "TPBoxBluePick": [0.5190224051475525, 0.7541279792785645, -1.9560778141021729, 2.019117593765259, -0.14910772442817688, -2.4870340824127197],
    
    "TPBoxYellowSetTop": [0.6688245951188749, 0.6343983456709559, -1.6915284600562606, 0.38543689492335353, -0.2980473372469321, -0.9574436162299582],
    "TPBoxYellowSet": [0.6691320538520813, 0.737246572971344, -1.6927298307418823, 0.572803795337677, -0.20657208561897278, -1.1514915227890015],
    "TPBoxYellowPick": [0.721162736415863, 0.7320846915245056, -2.023696184158325, 2.403364419937134, -0.1797034591436386, -2.5212976932525635],
    
    "TPBoxAlmSetTop": [-0.49951170702771464, 0.7397331423279123, -1.491397556595759, 0.25597026602765727, -0.40021876265482526, -0.37693350273204995],
    "TPBoxAlmSet": [-0.4997613728046417, 0.8315556645393372, -1.4941189289093018, 0.32541871070861816, -0.3099360466003418, -0.4515325427055359],
    "TPBoxAlmPick": [-0.4429081976413727, 0.7858123779296875, -1.8545527458190918, 1.3907990455627441, -0.1257384866476059, -1.5841832160949707],

    "P11": [0.24068158864974976, 0.412740558385849, -1.9809503555297852, 0.00015339808305725455, -0.7486186027526855, -0.2410619705915451]
}

def main():
    global Driver
    global robot
    global manipulator
    
    # 原点に移動
    manipulator.set_joint_value_target(PosisionsData["Home"])
    manipulator.go()
    
    while Driver.TPSupply(): # 収納レーンにワークがあれば
        
        # 収納レーンからピック
        manipulator.set_joint_value_target(PosisionsData["TPReturnPickTop"])
        manipulator.go()
        # ハンドを開く
        Driver.HandOpen()
        manipulator.set_joint_value_target(PosisionsData["TPReturnPick"])
        manipulator.go()
        # ハンドを閉じる
        Driver.HandCloses()
        
        # カラーセンサー台に設置
        manipulator.set_joint_value_target(PosisionsData["TPColorSetTop"])
        manipulator.go()
        manipulator.set_joint_value_target(PosisionsData["TPColorSet"])
        manipulator.go()
        
        # 色を判別
        Color = Driver.ColorSensor()
        if ColorTypeResult == ColorType.RED: # 赤ボックスに格納
            manipulator.set_joint_value_target(PosisionsData["TPBoxRedSetTop"])
            manipulator.go()
            manipulator.set_joint_value_target(PosisionsData["TPBoxRedSet"])
            manipulator.go()
        if ColorTypeResult == ColorType.BLUE: # 青ボックスに格納
            manipulator.set_joint_value_target(PosisionsData["TPBoxBlueSetTop"])
            manipulator.go()
            manipulator.set_joint_value_target(PosisionsData["TPBoxBlueSet"])
            manipulator.go()
        if ColorTypeResult == ColorType.YELLOW: # 黃ボックスに格納
            manipulator.set_joint_value_target(PosisionsData["TPBoxYellowSetTop"])
            manipulator.go()
            manipulator.set_joint_value_target(PosisionsData["TPBoxYellowSet"])
            manipulator.go()
        if ColorTypeResult == ColorType.ALUMINUM: # アルミボックスに格納
            manipulator.set_joint_value_target(PosisionsData["TPBoxAlmSetTop"])
            manipulator.go()
            manipulator.set_joint_value_target(PosisionsData["TPBoxAlmSet"])
            manipulator.go()
        if ColorTypeResult == ColorType.UNKNOWN: # エラー・回帰
            rospy.loginfo("ERROR!")
        
        # 原点に移動
        manipulator.set_joint_value_target(PosisionsData["Home"])
        manipulator.go()

if __name__ == "__main__":
    rospy.init_node("sequence_work_node")
    Driver = rs007n_commander.DriverCommander()
    robot = moveit_commander.RobotCommander()
    robot.get_group_names()
    manipulator = moveit_commander.MoveGroupCommander('manipulator')
    main()
