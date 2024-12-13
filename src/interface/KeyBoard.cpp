#include "../../include/interface/KeyBoard.h"
#include <iostream>

template<typename T>
inline T max(const T a, const T b){
	return (a > b ? a : b);
}

template<typename T>
inline T min(const T a, const T b){
	return (a < b ? a : b);
}

KeyBoard::KeyBoard(){
    userCmd = UserCommand::NONE;
    userValue.setZero();

    tcgetattr( fileno( stdin ), &_oldSettings );
    _newSettings = _oldSettings;
    _newSettings.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr( fileno( stdin ), TCSANOW, &_newSettings );

    int result = pthread_create(&_tid, NULL, runKeyBoard, (void*)this);
    std::cout << "result = " << result << std::endl;
    if (result != 0) {
        std::cerr << "Error creating keyboard thread: " << strerror(result) << std::endl;
    }
    // abort();
}

KeyBoard::~KeyBoard(){
    pthread_cancel(_tid);
    pthread_join(_tid, NULL);
    tcsetattr( fileno( stdin ), TCSANOW, &_oldSettings );
}

UserCommand KeyBoard::checkCmd(){
    switch (_c){
    // case ' ':
    //     return UserCommand::EXIT;
    case '1':
        return UserCommand::L2_X; // from FSMPassive to FSMWalking
    case '2':
        return UserCommand::L2_B; // from FSMWalking to FSMPassive
    case '3':
        return UserCommand::L2_A; // no effect
    case '4':
        return UserCommand::START; // no effect
    case ' ':
        // userValue.setZero();
        return UserCommand::NONE;
    default:
        return UserCommand::NONE;
    }
}

UserCommand KeyBoard::checkGait(){
    switch (_c){
    case '3':
        return UserCommand::WALK; // walking
    case '4':
        return UserCommand::STAND; // standing
    case ' ':
        return UserCommand::NONE;
    default:
        return UserCommand::NONE;
    }
}

void KeyBoard::changeValue(){
    switch (_c){
    case 'w':case 'W':
        userValue.ly = min<float>(userValue.ly+sensitivityLeft, 1.f);
        std::cout << "command velocity (vx, vy, wz): " << userValue.lx << " " << userValue.ly << " " << userValue.rx << " " << userValue.ry << std::endl;
        break;
    case 's':case 'S':
        userValue.ly = max<float>(userValue.ly-sensitivityLeft, -1.f);
        std::cout << "command velocity (vx, vy, wz): " << userValue.lx << " " << userValue.ly << " " << userValue.rx << " " << userValue.ry << std::endl;
        break;
    case 'd':case 'D':
        userValue.lx = min<float>(userValue.lx+sensitivityLeft, 1.f);
        std::cout << "command velocity (vx, vy, wz): " << userValue.lx << " " << userValue.ly << " " << userValue.rx << " " << userValue.ry << std::endl;
        break;
    case 'a':case 'A':
        userValue.lx = max<float>(userValue.lx-sensitivityLeft, -1.f);
        std::cout << "command velocity (vx, vy, wz): " << userValue.lx << " " << userValue.ly << " " << userValue.rx << " " << userValue.ry << std::endl;
        break;
    // ignore the following commands
    case 'i':case 'I':
        userValue.ry = min<float>(userValue.ry+sensitivityRight, 1.f);
        std::cout << "command velocity (vx, vy, wz): " << userValue.lx << " " << userValue.ly << " " << userValue.rx << " " << userValue.ry << std::endl;
        break;
    case 'k':case 'K':
        userValue.ry = max<float>(userValue.ry-sensitivityRight, -1.f);
        std::cout << "command velocity (vx, vy, wz): " << userValue.lx << " " << userValue.ly << " " << userValue.rx << " " << userValue.ry << std::endl;
        break;
    /////////////////////////////////
    case 'l':case 'L':
        userValue.rx = min<float>(userValue.rx+sensitivityRight, 1.f);
        std::cout << "command velocity (vx, vy, wz): " << userValue.lx << " " << userValue.ly << " " << userValue.rx << " " << userValue.ry << std::endl;
        break;
    case 'j':case 'J':
        userValue.rx = max<float>(userValue.rx-sensitivityRight, -1.f);
        std::cout << "command velocity (vx, vy, wz): " << userValue.lx << " " << userValue.ly << " " << userValue.rx << " " << userValue.ry << std::endl;
        break;
    default:
        break;
    }
}

void* KeyBoard::runKeyBoard(void *arg){
    ((KeyBoard*)arg)->run(NULL);
}

void* KeyBoard::run(void *arg){
    while(1){
        std::cout << "key test" << std::endl;
        // abort();
        FD_ZERO(&set);
        FD_SET( fileno( stdin ), &set );

        res = select( fileno( stdin )+1, &set, NULL, NULL, NULL);

        if(res > 0){
            read( fileno( stdin ), &_c, 1 );
            userCmd = checkCmd();
            gaitNum = checkGait();
            if(userCmd == UserCommand::L2_X){
                changeValue();
            }
            _c = '\0';
        }
        usleep(1000);
    }
}