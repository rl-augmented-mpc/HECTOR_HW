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
    if (result != 0) {
        std::cerr << "Error creating keyboard thread: " << strerror(result) << std::endl;
    }
}

KeyBoard::~KeyBoard(){
    pthread_cancel(_tid);
    pthread_join(_tid, NULL);
    tcsetattr( fileno( stdin ), TCSANOW, &_oldSettings );
}

UserCommand KeyBoard::checkCmd(){
    switch (_c){
    case '1':
        return UserCommand::PASSIVE;
    case '2':
        return UserCommand::PDSTAND; // from any mode to FSMPDStand
    case '3':
        return UserCommand::STAND; // from any mode to FSMWALK + standing gait
    case '4':
        return UserCommand::WALK; // from any mode to FSMWALK + walking gait
    case ' ':
        return UserCommand::NONE;
    default:
        return UserCommand::NONE;
    }
}

void KeyBoard::changeValue(){
    switch (_c){
    case 'w':
        userValue.lx = min<float>(userValue.lx+sensitivityLeft, 1.f);
        std::cout << "command velocity (vx, vy, wz): " << userValue.lx << " " << userValue.ly << " " << userValue.rx << " " << userValue.ry << std::endl;
        break;
    case 's':
        userValue.lx = max<float>(userValue.lx-sensitivityLeft, -1.f);
        std::cout << "command velocity (vx, vy, wz): " << userValue.lx << " " << userValue.ly << " " << userValue.rx << " " << userValue.ry << std::endl;
        break;
    case 'a':
        userValue.ly = min<float>(userValue.ly+sensitivityLeft, 1.f);
        std::cout << "command velocity (vx, vy, wz): " << userValue.lx << " " << userValue.ly << " " << userValue.rx << " " << userValue.ry << std::endl;
        break;
    case 'd':
        userValue.ly = max<float>(userValue.ly-sensitivityLeft, -1.f);
        std::cout << "command velocity (vx, vy, wz): " << userValue.lx << " " << userValue.ly << " " << userValue.rx << " " << userValue.ry << std::endl;
        break;
    case 'q':
        userValue.rx = min<float>(userValue.rx+sensitivityRight, 1.f);
        std::cout << "command velocity (vx, vy, wz): " << userValue.lx << " " << userValue.ly << " " << userValue.rx << " " << userValue.ry << std::endl;
        break;
    case 'e':
        userValue.rx = max<float>(userValue.rx-sensitivityRight, -1.f);
        std::cout << "command velocity (vx, vy, wz): " << userValue.lx << " " << userValue.ly << " " << userValue.rx << " " << userValue.ry << std::endl;
        break;
    default:
        break;
    }
}

void* KeyBoard::runKeyBoard(void *arg){
    ((KeyBoard*)arg)->run(NULL);
    return nullptr;
}

void* KeyBoard::run(void *arg){
    while(1){
        FD_ZERO(&set);
        FD_SET( fileno( stdin ), &set );

        res = select( fileno( stdin )+1, &set, NULL, NULL, NULL);

        if(res > 0){
            ssize_t bytesRead = read( fileno( stdin ), &_c, 1 );
            if (_c >= '1' && _c <= '4') {
                    userCmd = checkCmd(); // Update userCmd only for command keys
                }
            if(userCmd == UserCommand::WALK){
                changeValue();
            }
            _c = '\0';
        }
        usleep(1000);
    }

    return nullptr;
}