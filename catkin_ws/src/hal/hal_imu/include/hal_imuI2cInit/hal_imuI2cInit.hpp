#ifndef HAL_IMUI2CINIT
#define HAL_IMUI2CINIT

bool getHandle(hal_imu::hal_imuGetHandle::Request &req,
               hal_imu::hal_imuGetHandle::Response &res);

void initI2cCommunication(void);

class ImuI2cInit
{
private:
public:
    ImuI2cInit();
    ~ImuI2cInit();
    void initI2cCommunication(void);
};

#endif