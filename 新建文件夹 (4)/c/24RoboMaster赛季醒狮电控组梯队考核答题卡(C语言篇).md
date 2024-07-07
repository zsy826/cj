# ***24RoboMaster醒狮电控组梯队考核答题卡**
>-------------------------------------------------------------------------------------------------------------------------***C语言篇***


## PART I
```c
Q1.D
```

```c
Q2.B
```

```c
Q3.C
```

```c
Q4.C
```


```c
Q5.	A
```

```c
Q6.B
```

```c
Q7.6
```

```c
Q8.B
```

```c
Q9.D
```

```c
Q10.break语句主要用于结束当前循环或者条件语句，并跳出该循环或者条件语句
    continue语句主要用于跳过当前循环的剩余代码，并进入下一次循环
```
## PART II
### ***Q11:举例出C语言中的数据类型（越多越好）***
```c
int, short, long,  long long
float, double ， long double    
char，Bool,enum,struct,union,int *p
    
```


### ***Q12:使用 `while`自定义一个死循环函数***
```c
#include <stdio.h>  
  
void  loop() {  
    while(1) {  
        printf("1234\n");  
    }  
}  
  
int main() {  
    loop();  
    return 0;  
}
```
### ***Q13：解释 `VAL_LIMIT`  ，并计算 `Alpha` 、`Beta`、`Gamma` 的最终值***
```c
这个代码段中定义了一个宏 VAL_LIMIT ，它是一个预处理器宏，用于在编译时进行值检查。它接受三个参数：要检查的值（val），最小允许值（min）和最大允许值（max）。它接受三个参数：要检查的值（val），最小允许值（min）和最大允许值（max）。如果值（val）小于或等于最小值（min），则将值设置为最小值；如果值大于或等于最大值（max），则将值设置为最大值。
    Alpha 的值保持不变，为 1.0 Beta 的值被设置为最大值 43。Gamma 的值被设置为最小值 -19
```

### ***Q14:于 `笛卡尔坐标系` 中表示 `now1`与 `now2`的`变化曲线`***
```c

```

### ***Q15:请`自定义变量`，并巧妙利用 `ramp_t`, `ramp_init`, `ramp_calc` 实现`自定义变量` 的 `斜坡式下降`与`斜坡式增长`***
```c
#include <stdio.h>
#include <stdint.h>

typedef struct
{
  uint32_t get_count;
  uint32_t set_count;
  float out;
} ramp_t;

void ramp_init(ramp_t *ramp, uint32_t target_count)
{
  ramp->get_count = 0;
  ramp->set_count = target_count;
  ramp->out = 0;
}

float ramp_calc(ramp_t *ramp)
{
  if (ramp->set_count <= 0)
    return 0;
  if (ramp->get_count >= ramp->set_count)
    ramp->get_count = ramp->set_count;
  else
    ramp->get_count++;

  ramp->out = (float)ramp->get_count / (float)ramp->set_count;
  return ramp->out;
}

int main()
{
  ramp_t myVariable; 

  ramp_init(&myVariable, 10); 

  for (int i = 0; i < 20; i++)
  {
    float result = ramp_calc(&myVariable); 
    printf("Iteration %d: %f\n", i + 1, result);
  }

  return 0;
}

```

### ***Q16:现有名为 `HAL_GetTick` 的时间测量函数,其单位为 `ms,请利用 `HAL_GetTick`函数返回值的特殊性质,实现程序 `1ms`定时打印`"Hello,world !"` ( tip:① 定义变量与函数配合获得∆t值；② if语句)***
```c
#include <stdio.h>
static uint32_t lastPrintTime = 0;

void printHelloWorld()
{
    
    uint32_t currentTime = HAL_GetTick();

    uint32_t deltaTime = currentTime - lastPrintTime;

    if (deltaTime >= 1)
    {
        printf("Hello, world!\n");

        lastPrintTime = currentTime;
    }
}

int main()
{
    while (1)
    {
        
        printHelloWorld();
    }

    return 0;
}

```
### ***Q17:使用名为 `callback` 的 `函数指针` 初始化 `alpha`与 `beta` 的值***
```c
#include <stdio.h>

#define VALUE_B 217
#define VALUE_C 219.321

static int alpha;
static float beta;

void STDRxCallback(int* b, float* c)
{
    *b = VALUE_B;
    *c = VALUE_C;
}

void (*callback)(int* c, float* b) = STDRxCallback;

int main()
{
    
    callback(&alpha, &beta);

    
    printf("alpha: %d\n", alpha);
    printf("beta: %.3f\n", beta);

    return 0;
}

```

### ***Q18:请你初始化 `alpha `的值为`` gama`` 的`低八位 `、`beta `的值为` gama` 的`高八位`***
```c
#include<stdio.h>
#include<stdint.h>
int main(){
	uint16_t gama = 20001;
	unit8 t alpha,beta;
	alpha=(gama&0xFF00) >>8;
	beta=gama&0x00FF;
	printf("alpha:%u n,beta:%u",alpha,beta);
    return 0;
}
```

### ***∆Q19:下图代码为PID控制算法公式 ,请 `自定义结构体` ,并利用 `pid_t`、`PID_Struct_Init`、`pid_calc` 分别初始化 `kp`、`ki`、`kd`、`maxout`、`integral_limit` 的值  , 并得到经 `pid_calc`计算后的`最终值 `(tip: 可令 kp = 1 ,ki = 0.01,kd = 20 ,maxout = 500,integral_limit = 100 并将其传入 PID_Struct_Init 进行初始化,最后调用 pid_calc 得到最终值)`***
```c
typedef struct pid
{
    float set;
    float get;
    float error[3];

    float kp;
    float ki;
    float kd;

    float pout;
    float iout;
    float dout;
    float out;

    int32_t maxout;
    int32_t integral_limit;

    void (*f_pid_init)(struct pid *pid_t,
                       float p,
                       float i,
                       float d,
                       int32_t max_out,
                       int32_t integral_limit);
    void (*f_pid_reset)(struct pid *pid_t,
                        float p,
                        float i,
                        float d);

} pid_t;

static void abs_limit(float *x, int32_t limit)
{
    if (*x > limit)
        *x = limit;
    if (*x < -limit)
        *x = -limit;
}

void pid_init(pid_t *pid, float p, float i, float d, int32_t max_out, int32_t integral_limit)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxout = max_out;
    pid->integral_limit = integral_limit;
}

void pid_reset(pid_t *pid, float p, float i, float d)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;

    pid->pout = 0;
    pid->iout = 0;
    pid->dout = 0;
    pid->out = 0;
}

void PID_Struct_Init(pid_t *pid, float p, float i, float d, int32_t max_out, int32_t integral_limit)
{
    pid->f_pid_init = pid_init;
    pid->f_pid_reset = pid_reset;

    pid->f_pid_init(pid, p, i, d, max_out, integral_limit);
    pid->f_pid_reset(pid, p, i, d);
}

float pid_calc(pid_t *pid, float get, float set)
{
    pid->get = get;
    pid->set = set;
    pid->error[NOW_ERR] = set - get;

    pid->pout = pid->kp * pid->error[NOW_ERR];
    pid->iout += pid->ki * pid->error[NOW_ERR];
    pid->dout = pid->kd * (pid->error[NOW_ERR] - pid->error[LAST_ERR]);
    // 积分限幅
    abs_limit(&(pid->iout), pid->integral_limit);
    pid->out = pid->pout + pid->iout + pid->dout;
    abs_limit(&(pid->out), pid->maxout);
    // 最终值限幅
    pid->out += pid->pout + pid->iout + pid->dout;
    abs_limit(&(pid->out), pid->maxout);

    pid->error[LLAST_ERR] = pid->error[LAST_ERR];
    pid->error[LAST_ERR] = pid->error[NOW_ERR];
    // 最终值输出
    return pid->out;
}

int main()
{
 
    pid_t my_pid;
    PID_Struct_Init(&my_pid, 1, 0.01, 20, 500, 100);

    
    float get_value = 10.0; 
    float set_value = 5.0;  
    float final_value = pid_calc(&my_pid, get_value, set_value);

    printf("Final value: %.2f\n", final_value);

    return 0;
}

```

## PART III
### ***Q20：题目：请自定义一个结构体类型 `"Student"`，包含以下成员变量` (特殊要求：用结构体与指针相关内容解决)`***

```c
姓名（字符串类型）
学号（整型）
成绩（单精度浮点型）
请编写一个函数，接收一个指向 "Student" 结构体数组的指针，将每个学生的姓名、学号和成绩输出到屏幕上。
```
### ***Q21：请编写一个函数，接收一个整型数组和数组长度作为参数，找出数组中的最大值，并通过指针将最大值返回给调用函数。`(特殊要求:利用结构体与指针相关内容解决)`***
```c
#include <stdio.h>
 struct MaxResult {
    int maxValue};

int main() {
    int array[] = {8, 11, 4, 2, 6};
    int length = sizeof(array) / sizeof(array[0]);
	int max;
   

void findMaxValue(int *arr, int length, struct MaxResult *result) {
    int max = arr[0];
    for (int i = 1; i < length; i++) {
        if (arr[i] > max) {
            max = arr[i];
        }
    }
    result->maxValue = max;
}
    struct MaxResult maxResult;
    findMaxValue(array, length, &maxResult);

    printf("最大值是：%d\n", maxResult.maxValue);

    return 0;
}

```


### ***Q22：请编写一个void函数，使其限制电机转速在-4000~4000范围内 `(特殊要求:在不用全局变量的情况下完成 )`***
```c
#include <stdio.h>
void LimitSpeed(int nowspeed);
int main() {
    int nowspeed=5000,maxspeed=4000;
    
	LimitSpeed(nowspeed);
   
    printf("nowspeed的值",nowspeed)
    return 0;
}
void LimitSpeed(int nowspeed)
{	if(nowspeed > 4000)
	{	nowspeed = 4000;
    }
 	else if(nowspeed<-4000)
    {	  
    		nowspeed=-4000;}
 
```

### ***Q23：已知 `云台` 有四种状态，分别为`GIMBAL_INIT_NEVER`，`GIMBAL_INIT_DONE，NO_ACTION`，`IS_ACTION` 四种，请你定义一个`枚举结构体gimbal_state_t`放入这四种状态；假设`现在`云台状态为`GIMBAL_INIT_DONE`，请你写出程序,使其输出`当前状态`的枚举值 `(特殊要求:利用枚举)`***
```c
#include <stdio.h>

typedef enum {
    GIMBAL_INIT_NEVER,
    GIMBAL_INIT_DONE,
    NO_ACTION,
    IS_ACTION
} gimbal_state_t;

int main() {
    gimbal_state_t current_state = GIMBAL_INIT_DONE;
    
    printf("当前状态的枚举值为: %d\n", current_state);
    
    return 0;
}
```
## PART IV
### ***Q24：请定义一个`枚举类型"CourseType"`，包含以下选项：`数学`、`英语`、`物理`、`化学`。定义一个`结构体类型"Course"`，包含以下成员：`课程类型（type）：枚举类型"CourseType"` ;`学分（credit）：整数类型` ;`成绩（score）：浮点数类型` 。并声明一个`指向"Course"结构体的指针"courses"` 。编写一个`函数"addCourse()"`，该函数接收用户输入的`课程类型、学分和成绩`，并将其添加到`"courses"所指向的结构体数组`中。如果数组已`满`(组数为10)，则提示`错误信息`。编写一个`函数"calculateGPA()"`，该函数接收`"courses"指针和数组长度作为参数`，计算所有课程的平均学分绩点（GPA）。学分绩点按照以下规则计算：90-100分为A，4.0绩点；80-89分为B，3.0绩点；70-79分为C，2.0绩点；60-69分为D，1.0绩点；低于60分为F，0绩点。在主函数中，首先让用户输入课程数量，调用"addCourse()"函数让用户依次输入课程信息，最后`调用"calculateGPA()"函数`计算`平均学分绩点`并`打印`出来。***
```c

```

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>

<br>









​								  ***@醒狮机器人实验室电控组***

