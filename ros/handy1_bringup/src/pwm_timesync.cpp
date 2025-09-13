#include <stdio.h>
#include <unistd.h>
#include <pigpio.h>
#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <csignal>
#include <atomic>

// g++ -o sync mid360_gs_sync.cpp -lpigpio -lrt -lpthread

const int PPS_INPUT=6;
const int PPS_OUTPUT=12;
const int SHUTTER=13;

uint32_t lasttick = 0;
struct::timespec laststamp;
int fd;

void gpio_callback(int gpio, int level, uint32_t tick, void *user) {
    if (level == 1){
        struct::timespec stamp;
        clock_gettime(CLOCK_MONOTONIC, &stamp);
        int64_t stampdiff = (stamp.tv_sec*1000000000 + stamp.tv_nsec)-(laststamp.tv_sec*1000000000 + laststamp.tv_nsec);
        stampdiff /= 1000;

        lasttick = tick;
        laststamp.tv_sec = stamp.tv_sec;
        laststamp.tv_nsec = stamp.tv_nsec;

        // 現在時刻を取得してNMEAセンテンスを生成
        time_t now = time(nullptr);
        struct tm *utc = gmtime(&now);

        char msg[128];
        // UTC時刻をhhmmss形式で
        char timebuf[10];
        // snprintf(timebuf, sizeof(timebuf), "%02d%02d%02d", utc->tm_hour, utc->tm_min, utc->tm_sec);
        // ミリ秒の上2桁を取得
        struct timespec now_ts;
        clock_gettime(CLOCK_REALTIME, &now_ts);
        int ms = now_ts.tv_nsec / 1000000;
        int ms2 = ms / 10; // 上2桁
        snprintf(timebuf, sizeof(timebuf), "%02d%02d%02d.%02d", utc->tm_hour, utc->tm_min, utc->tm_sec, ms2);

        // printf("%s\n", timebuf);

        // 日付をddmmyy形式で
        char datebuf[7];
        snprintf(datebuf, sizeof(datebuf), "%02d%02d%02d", utc->tm_mday, utc->tm_mon + 1, (utc->tm_year + 1900) % 100);

        // 本体部分を作成
        char nmea_body[128];
        snprintf(nmea_body, sizeof(nmea_body),
            "GPRMC,%s,A,2812.0498,N,11313.1361,E,0.0,180.0,%s,3.9,W,A",
            timebuf, datebuf);

        // チェックサム計算
        unsigned char checksum = 0;
        for (size_t i = 0; nmea_body[i] != '\0'; ++i) {
            checksum ^= nmea_body[i];
        }

        // センテンス全体を作成
        snprintf(msg, sizeof(msg), "$%s*%02X\r\n", nmea_body, checksum);
        int len = strlen(msg);
        int written = write(fd, msg, len);
        if (written < 0) {
            std::cerr << "UART書き込み失敗" << std::endl;
        } else {
            // std::cout << "UART送信: " << msg;
        }
    }
}

std::atomic<bool> interrupted(false);
void signalHandler(int signum)
{
    std::cout << "signalHandler: received signal " << signum << '\n';
    interrupted = true;
}

int main()
{
    gpioCfgInterfaces(PI_DISABLE_SOCK_IF);
    if (gpioInitialise() < 0) {
        fprintf(stderr, "pigpioの初期化に失敗しました。\n");
        return 1;
    }

    const char* device = "/dev/serial0"; // 標準UARTデバイス
    fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "UARTオープン失敗" << std::endl;
        return 1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B9600); // ボーレート設定
    cfsetospeed(&options, B9600);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB; // パリティなし
    options.c_cflag &= ~CSTOPB; // 1ストップビット
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;     // 8ビット
    options.c_cflag &= ~CRTSCTS; // フロー制御なし
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 非カノニカル
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // フロー制御なし
    options.c_oflag &= ~OPOST; // 生出力
    tcsetattr(fd, TCSANOW, &options);

    int freq = 10;
    int duty = 350000;

    struct::timespec time1, time2, time3;
    clock_gettime(CLOCK_MONOTONIC, &time1);
    clock_gettime(CLOCK_MONOTONIC, &time2);

    gpioHardwarePWM(SHUTTER, 10, 100000);
    clock_gettime(CLOCK_MONOTONIC, &time3);
    gpioHardwarePWM(PPS_OUTPUT, 1, 25000); //ここが33.3usズレる

    double sec1 = (time1.tv_sec + time1.tv_nsec*1e-9) *1000;
    double sec2 = (time2.tv_sec + time2.tv_nsec*1e-9) *1000;
    double sec3 = (time3.tv_sec + time3.tv_nsec*1e-9) *1000;

    printf("time1: %lf\n", sec1);
    printf("time2: %lf\n", sec2-sec1);
    printf("time3: %lf\n", sec3-sec1);

    clock_gettime(CLOCK_MONOTONIC, &laststamp);
    gpioSetMode(PPS_INPUT, PI_INPUT);
    gpioSetPullUpDown(PPS_INPUT, PI_PUD_DOWN); // プルダウン

    // 割り込み設定（両エッジ検出）
    gpioSetAlertFuncEx(PPS_INPUT, gpio_callback, nullptr);

    std::signal(SIGINT, signalHandler);

    for (;;) {
        if (interrupted) break;
        sleep(1);
    }

    gpioHardwarePWM(SHUTTER, 0, 0);
    gpioSetMode(SHUTTER, PI_OUTPUT);
    gpioWrite(SHUTTER, 0);

    gpioHardwarePWM(PPS_OUTPUT, 0, 0);
    gpioSetMode(PPS_OUTPUT, PI_OUTPUT);
    gpioWrite(PPS_OUTPUT, 0);

    gpioTerminate();
    close(fd);
    printf("終了\n");
    
    return 0;
}