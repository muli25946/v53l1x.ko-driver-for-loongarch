#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "sys/ioctl.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"
#include <poll.h>
#include <sys/select.h>
#include <sys/time.h>
#include <signal.h>
#include <fcntl.h>

/*
 * @description		: main主程序
 * @param - argc 	: argv数组元素个数
 * @param - argv 	: 具体参数
 * @return 			: 0 成功;其他 失败
 */
int main(int argc, char *argv[])
{
    int fd;
    char *filename;
    unsigned short distance_mm;
    int ret = 0;

    if (argc != 2)
    {
        printf("Error Usage!\r\n");
        return -1;
    }

    filename = argv[1];
    fd = open(filename, O_RDWR);
    if (fd < 0)
    {
        printf("can't open file %s\r\n", filename);
        return -1;
    }

    int i = 0;
    while (i < 10)
    {

        ret = read(fd, &distance_mm, sizeof(distance_mm));
        if (ret == 0)
        { /* 数据读取成功 */
            printf("distance_mm = %d mm\r\n", distance_mm);
        }
        else
        {
            printf("read error, ret = %d\r\n", ret);
        }
        i++;
        sleep(1);
    }
    close(fd); /* 关闭文件 */

    return 0;
}
