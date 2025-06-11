import numpy as np
import matplotlib.pyplot as plt
from environment import env
from robot import bot

if __name__ == '__main__':
#Welcome message.
    print('Welcome to M-bot Simulator \nYou will never taste an experience like this.')

#It requested the datas until the user inserts validy datas.
    while True: 
        try:
#Receving the informations about the environment (i.e., row and column) from the user.
            row = int(input('\nInsert the number of environments row: '))
            column = int(input('Insert the the number of environments column: '))
#Checking if the inserted data are validy.
            if row <=0 or column <= 0:
#Message of erro if the user inserts any invalidy data, i.e., row or column <= 0.
                print('\nInvalidy data! \nThe number of environments row or columns must be positives.\nTry again!')
            else:
#ending the loop.
                break 
#Checking if the inserted data are integers.
        except ValueError:
#Message of erro if the user inserts any invalidy data, i.e., they are not integers.
            print('\nInvalidy data! \nThe number of environments row or columns must be integers.\nTry again!')

#Creating the environment with the inserted data.
    real_env = env(row, column)
    while True:
        """Asking the user would like to insert or remove any obstacle, if the answer == 'no', it continued process. if the answer == 'yes',
then the user should insert the region which he wish to change it, following a procedure analogue to the previous, i.e., it check if the
entry integers in the right interval."""
        ans_insert_remove_obstacle = input('\nWould you like to insert/remove any static obstacle? (yes/no)\n')
        if ans_insert_remove_obstacle == 'yes':
#Receveing the type of obstacle, when the user insert k =0, this mean he wish to insert an obstacle, if k isn't 0, it remove an obstacle.
            k = input('\nInsert the type of application:\n\n(1) 0 to Insert obstacle;\n(2) Anything to remove obstacle.\n\n')
            while True:
#Explanation of the types of obstacle inserted/removed at a single time.
                print('\nThe M-bot simulator accepts 4 types of static obstacles to insertion and removal at a single time: \n\n(1) A single element [row,colum];\n(2) A single row;\n(3) A single column;\n(4) A region rectangle.\n')
                try:
#Receiving the region data.
                    initial_row = int(input('Insert the initial row (0-{}):\n' .format(row-1)))
                    initial_column = int(input ('Insert the initial column (0-{}):\n' .format(column-1)))
                    final_row = int(input('Insert the final row ({}-{}):\n' .format(initial_row, row-1)))
                    final_column= int(input('Insert the final column ({}-{}):\n' .format(initial_column, column -1)))
#It's checking if the entries are in the right interval, i.e., if the region mentioned above exist in the map, then it adds/removes 
#obstacles in the region.
                    if row > final_row >= initial_row and column > final_column >= initial_column:
                        real_env.k_obstacle([initial_row, final_row+1],[initial_column, final_column +1], k)
                        print('\nMap has been changed sucessfully!')
                        break
                    else:
#In case the region mentioned above does not exist in the map, it continues the loop.
                        print('\nInvalidy data!\nOut of domain elments in the map.\nTry again!\n')
                except ValueError:
                    print('\nInvalidy data!\nThey must be integers\nTry again!\n')
#If the answer == 'no', the it ends the loop.
        elif ans_insert_remove_obstacle == 'no':
            break
        else:
#If the user's answer is different of 'yes' or 'no', i.e., an answer different than expected.
            print('\nUnidentified answer.\nTry again to proceed!\n')
#Number of free positions in the map.
    num_free_positions = np.count_nonzero(np.array(real_env.map) == 0)
#It requested the datas until the user insert validy data.
    while True:
        try:
            num_mob_obstacles = int(input('\nHow many mobile obstacles would you like to insert?\n'))
#We limited the number of mobile obstacles with the number free positions and 2 positions, i.e., initial and final position.
            if num_mob_obstacles < 0 or num_mob_obstacles > num_free_positions -2:
#Message of erro if the user inserts any invalidy data, i.e.,it is not a positive integer or it takes up a greater amount than the free entries.
                print('\nInvalidy data!\nIt is not a positive integer or it takes up a greater amount than the free entries.\nTry again!')
            else:
#Adding the mobile obstacles in the env map
                real_env.mob_obstacles(num_mob_obstacles)
                break
        except ValueError:
            #Message of erro if the user inserts any invalidy data, i.e., it is not integer.
            print('\nInvalidy data!.\nTry again!')

#It requested the datas until the user insert validy data.
    while True:
        try:
            sensor_field = int(input('\nInsert the sensor field:\n(OBSERVATION: It must be an integer greater than or equal to 1.)\n'))
#Checking if the data is validy.
            if sensor_field >= 1:
#Creating the robot.
                robot = bot(sensor_field)
                break
            else:
#Message of erro if the user inserts any invalidy data, i.e., the data isn't an integer greater than 1.
                print('\nInvalidy data! \nIt must be an integer greater than 1.\nTry again!')
        except ValueError:
#Message of erro if the user inserts any invalidy data, i.e., the data isn't an integer.
            print('\nInvalidy data! \nIt must be an integer.\nTry again!')

#It requested the datas until the user insert validy datas.
    while True: 
        try:
#Receving the informations about the positions (i.e., initial and final positions) from the user.
            initial_position_str = input('\nInsert the initial position (separated by space): ').split()
            final_position_str = input('\nInsert the final position (separated by space): ').split()
#Converting the string lists to integer lists
            initial_position = [int(element) for element in initial_position_str]
            final_position = [int(element) for element in final_position_str]
#Receving the positions and checking if it correspond a free 2D position in the map.
            if (len(initial_position), len(final_position)) == (2,2):
                if 0<=initial_position[0]<row and 0<= initial_position[1]<column and 0<= final_position[0]<row and 0<= final_position[1]< column:
                    erro = []
                    robot.choose_position(initial_position, real_env, 0)
                    erro.append(robot.erro)
                    robot.choose_position(final_position, real_env, 1)
                    erro.append(robot.erro)
                    if erro == [0,0]:
                        break
                    else:
#Message of erro if the user inserts any invalidy data, i.e.,they are not free.
                     print('\nInvalidy data!\nThe position is not free.\nTry again!')
                else:
#Message of erro if the user inserts any invalidy data, i.e., the positions don't belong the map.
                    print('\nInvalidy data!\nThe position does not belong the map.\nTry again!')
            else:
#Message of erro if the user inserts any invalidy data, i.e., it does not belong the 2D plan
                print('\nInvalidy data!\nThe position does not belong the 2D plan.\nTry again!')
#Checking if the inserted data are correct type.
        except ValueError:
#Message of erro if the user inserts any invalidy data, i.e., they are not integers.
            print('\nInvalidy data!.\nTry again!')
#Doing the planning.
    robot.total_planning(real_env)
#Variable that indicates if the users wish to reamin in the app.
    continue_interactor = 1
    while continue_interactor == 1:
#It requested the datas until the user insert validy datas.
        while True:
            try:
                user_ans =int(input('\nSelected an option:\n(1)The environment map;\n(2)The robot displacement;\n(3)The historic planning.\n\n'))
                if user_ans == 1:
#Plotting only the environment
                    real_env.plotRealEnv()
                    break
                elif user_ans == 2 or user_ans == 3:
                    if user_ans == 2:
#Plotting the environment and the planning map
                        real_env.plotRealEnv()
                        robot.plotPlanningMap(robot.displacement)
                    else:
#Plotting each robot map and planned path
                        for element in robot.planned_historic:
                            element[0].plotEnv('bot')
                            robot.plotPlanningMap(element[1])
                    break
                else:
#Message of erro if the user inserts any invalidy data, i.e., it is not equal to 1,2 or 3.
                    print('\nType any valid integer, i.e., (1),(2) or (3).\n')
            except ValueError:
#Message of erro if the user inserts any invalidy data, i.e., it is not an integer.
                print('\nInvalidy data! \nThe data must be any integer.\nTry again!\n')
#It requested the datas until the user insert validy datas.
        while True:
            try:
#Asking if the user would like to continue in the app.
                continue_ans = int(input ('\nType:\n(1)To continue;\n(0)Exit.\n\n'))
                if continue_ans == 0 or continue_ans == 1:
                    continue_interactor = continue_ans
#If yes, it cheking if exists any mobile obstacles and updating the mobile obstacles positions and calculating all other important things.
                    if num_mob_obstacles >0 and continue_interactor == 1:
                        real_env.updating_mob_obstacles()
                        robot.total_planning(real_env)
                    break
                else:
#Message of erro if the user inserts any invalidy data, i.e., it is not equal 0 or 1.
                    print('\nType any valid integer, i.e., (0) or (1).\n')
            except ValueError:
#Message of erro if the user inserts any invalidy data, i.e., it is not an integer.
                print('\nInvalidy data! \nThe data must be any integer.\nTry again!\n')
#Thank you message for using the application and exit the app.
    print('\nThanks for using the M-bot Simulator.\nWe hope it has been helpful for you.\nHave a great day!\n')
    exit()