/*************************************/
/* RunACCM2025 track 模块 C 语言移植   */
/* 公共类型定义                        */
/*                                     */
/* 这里放所有模块共用的数据类型：       */
/*   POINT / PointList  点与点集        */
/*   Image             单通道灰度图      */
/*   PredictResult     AI 检测框         */
/*   Config            整车参数          */
/*   TaskData          一帧控制输出       */
/* 原工程对应：common.hpp、detection.hpp */
/* 的 PredictResult、param.hpp 的 Config。*/
/* Auther: Puking   也够而已啊，我   原谅我吧，只是小小的报复 */
/*************************************/
#ifndef TRACK_TYPES_H
#define TRACK_TYPES_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define COLSIMAGE 320 /* 图像宽（列数） */
#define ROWSIMAGE 240 /* 图像高（行数） */

/* 点集容量：替代 std::vector 的动态扩容。
   巡线单侧最多 180 点，600 足够放下中线、贝塞尔等中间结果。
   注意 PointList 是“按值”放进结构体的，一个 600 点的点集约 9.6 KB，
   Standard 里有几十个，所以 Standard 结构体约 483 KB。 */
#define LIST_CAP 600
#define POINTS_MAX_LEN 180 /* 迷宫寻线单侧最多走多少步，与原工程一致 */

   /* 舵机 PWM 范围：中值 3840，左右各留余量。
      下发前会 clip 到 [PWMSERVOMIN, PWMSERVOMAX]，防止打过机械限位。 */
#define PWMSERVOMAX 4700
#define PWMSERVOMID 3840
#define PWMSERVOMIN 3000

#define PI 3.1415926535898

      /* AI 标签，与原 detection.hpp 的 PredictLabel 顺序一致。
         注意：实际障碍检测代码里用的是原始数字（5 锥桶、7 行人、1 黑块），
         和这里的枚举值并不完全对应，移植时按原数字保留。 */
enum {
    LABEL_BATTERY = 0,  /* 充电站 */
    LABEL_BLOCK,        /* 障碍块 */
    LABEL_BRIDGE,       /* 坡道 */
    LABEL_BURGER,       /* 汉堡 */
    LABEL_CAR,          /* 道具车 */
    LABEL_COMPANY,      /* 公司 */
    LABEL_CONE,         /* 锥桶 */
    LABEL_CROSSWALK,    /* 斑马线 */
    LABEL_PEDESTRIAN,   /* 行人 */
    LABEL_SCHOOL        /* 学校 */
};

/* 二维点，对应原 POINT。
   x/y 是整数像素坐标；slope/angle 是后续算法填的附加信息
   （局部角度、拐点角度），普通点用不到。 */
typedef struct POINT {
    int   x;
    int   y;
    float slope;
    float angle;
} POINT;

/* 点集，替代 std::vector<POINT>：
   size 表示当前有效点数，push 时追加，clear 只把 size 置 0。
   下标访问用 p[i]，越界不会报错，调用方要自己保证。 */
typedef struct PointList {
    POINT p[LIST_CAP];
    int   size;
} PointList;

/* 单通道灰度图，替代 cv::Mat(CV_8UC1)。
   data 是一维连续内存，尺寸 rows*cols，像素 (x,y) 的下标是
   y*cols + x，用 IMG_AT(img, x, y) 取。 */
typedef struct Image {
    int            rows; /* 高 */
    int            cols; /* 宽 */
    unsigned char* data; /* 行优先存放，0 黑 255 白 */
} Image;

/* AI 检测结果，对应原 PredictResult：
   检测框左上角 (x, y)、宽高、类别 type、置信度 score。
   原来的 std::string label 在控制逻辑里没用，去掉了。 */
typedef struct PredictResult {
    int   type;
    float score;
    int   x;
    int   y;
    int   width;
    int   height;
} PredictResult;

/* 元素顺序表和参数表的容量，实际比赛元素不会超过 16 个 */
#define ORDER_CAP 16

/* 整车参数，对应原 param.hpp 的 Config（原来用 nlohmann::json 读文件，
   C 版改成结构体 + config_default() 填默认值，需要时直接改字段）。 */
typedef struct Config {
    /* 元素顺序：1 汉堡 2 虚线停车 4 障碍 5 圆环 6 十字 7 桥 0 终止 8~11 充电。
       处理完一个元素 order_index 加一，指向下一个 */
    int   elem_order[ORDER_CAP];
    int   elem_order_size;
    /* 与 elem_order 逐项对应：速度、近预瞄、P、D */
    int   speed_order[ORDER_CAP];
    int   speed_order_size;
    float aim_dis_n_order[ORDER_CAP];
    int   aim_dis_n_order_size;
    float aim_angle_p_order[ORDER_CAP];
    int   aim_angle_p_order_size;
    float aim_angle_d_order[ORDER_CAP];
    int   aim_angle_d_order_size;
    /* 每个元素是否做 AI 障碍检测：0 不检测，非 0 检测 */
    int   obstacle_order_index[ORDER_CAP];
    int   obstacle_order_index_size;

    /* ---- 速度 ---- */
    float speedLow;    /* 低速（特殊区域） */
    float speedHigh;   /* 高速（直道） */
    float speedBridge; /* 坡道 */
    float speedRing;   /* 环岛 */
    float speedCross;  /* 十字 */
    float speedDown;   /* 其他降速区 */

    /* ---- 预瞄距离（米）---- */
    float aim_distance_far;   /* 远预瞄 */
    float aim_distance_near;  /* 近预瞄 */
    float aim_distance_ring;  /* 环内 */

    /* ---- 环岛/舵机 PID ---- */
    float ring_entering_p_k; /* 进环时按偏差叠的 P 增量 */
    float ring_p_k;          /* 环内 P 增量 */
    float steering_p;
    float steering_d;
    float curve_p; /* 弯道单独参数，0 表示不用 */
    float curve_d;
    float ring_d;

    /* ---- 调试 / AI 开关 ---- */
    bool en_show; /* 显示调试图（C 版未实现显示，保留字段） */
    bool saveImg; /* 存图（C 版可用 img_save_pgm） */
    bool enAI;    /* 是否启用 AI 检测 */

    /* ---- 各元素专用参数 ---- */
    int CATERING_STOP_TIMESTAMP;          /* 汉堡停车时间戳 */
    int CHARGING_LEFT_FIRST_GOIN_TIME;    /* 左一车库进库计时 */
    int CHARGING_LEFT_SECOND_GOIN_TIME;   /* 左二 */
    int CHARGING_RIGHT_FIRST_GOIN_TIME;   /* 右一 */
    int CHARGING_RIGHT_SECOND_GOIN_TIME;  /* 右二 */
    int CHARGING_LEFT_FIRST_OUT_TIME_ADD;   /* 左一出库补偿 */
    int CHARGING_LEFT_SECOND_OUT_TIME_ADD;  /* 左二出库补偿 */
    int CHARGING_RIGHT_FIRST_OUT_TIME_ADD;  /* 右一出库补偿 */
    int CHARGING_RIGHT_SECOND_OUT_TIME_ADD; /* 右二出库补偿 */
    int CHARGING_GOIN_AIM_SPEED;          /* 进库速度 */
    int STOP_EXPECT_Y;                    /* 斑马线停车行 */

    int LAYBY_SHIFT_LEFT;    /* 临时停车：左边线平移量（像素） */
    int LAYBY_SHIFT_RIGHT;   /* 右边线平移量 */
    int LAYBY_PARKING_TIMER; /* 停车持续帧数 */
    int LAYBY_AIM_SPEED;     /* 停车区速度 */

    /* ---- 障碍物 ---- */
    int   OBSTACLE_width_conical;    /* 锥桶检测框宽阈值 */
    int   OBSTACLE_width_pedestrian; /* 行人检测框宽阈值 */
    int   OBSTACLE_width_block;      /* 障碍块检测框宽阈值 */
    int   block_aim_speed;           /* 黑块通过速度 */
    int   cone_aim_speed;            /* 锥桶通过速度 */
    float cone_p;                    /* 锥桶绕行 PID */
    float cone_d;
    float pe_p;                      /* 行人绕行 PID */
    float pe_d;
    float block_p;                   /* 黑块绕行 PID */
    float block_d;
} Config;

/* 一帧输出，对应原 TaskData（原结构里还有 cv::Mat 图像和时间戳，
   控制只需要舵机 PWM、目标速度、蜂鸣器）。 */
typedef struct TaskData {
    int    pwm;    /* 舵机 PWM，[PWMSERVOMIN, PWMSERVOMAX] */
    double speed;  /* 目标速度 */
    bool   buzzer; /* 是否鸣笛提示（进入元素时为 true） */
} TaskData;

/* 填一份与原工程参数默认值一致的配置 */
void config_default(Config* cfg);

#endif /* TRACK_TYPES_H */
