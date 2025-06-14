%% 清理工作区及命令行等
clear,clc,close all;

%% 定义所用函数及参量

% 定义全局变量
global position pencil positions home VALUE STEP s title
position = [0, 0, 0, 0, 0, 0];
title = {'X', 'Y', 'Z', 'a', 'b', 'c'};
positions = [];
home = position;

% 绘制画笔和初始化参数
pencil = [0, 1, 0, 0, 0, 0];
VALUE = 90;
STEP = 9;

s = sin(pi/36);

% 存储当前位置函数
function store()
    global position positions
    positions = [positions; position];
end

% 向前
function FD(value, step)
    global STEP VALUE position pencil
    % 设置默认值
    if nargin < 2
        step = STEP;
    end
    if nargin < 1
        value = VALUE;
    end

    if value > step
        for i = 1: step
            position = position + (value / step) * pencil;
            store();
        end
    else
        position = position + value * pencil;
        store();
    end
end

% 向后
function BK(value)
    global VALUE
    if nargin < 1
        value = VALUE;
    end

    FD(-value);
end

% 左转
function TL(value)
    global VALUE pencil
    if nargin < 1
        value = VALUE;
    end

    angle = pi * value / 180;
    x = pencil(1);
    y = pencil(2);
    z = pencil(3);

    Y = cos(angle) * y - sin(angle) * z;
    Z = cos(angle) * z + sin(angle) * y;

    pencil(1) = x;
    pencil(2) = Y;
    pencil(3) = Z;
end

% 右转
function TR(value)
    global VALUE
    if nargin < 1
        value = VALUE;
    end

    TL(-1 * value);
end

% 抬起笔尖
function TB(value, step)
    global position STEP VALUE
    % 设置默认值
    if nargin < 2
        step = STEP;
    end
    if nargin < 1
        value = VALUE;
    end

    for i = 1:step
        position(1) = position(1) - value / step;
        store();
    end
end

% 放下笔尖
function LB(value, step)
    global position STEP VALUE
    % 设置默认值
    if nargin < 2
        step = STEP;
    end
    if nargin < 1
        value = VALUE;
    end

    for i = 1:step
        position(1) = value / step + position(1);
        store();
    end
end

function RESET()
    global pencil
    pencil = [0, 1, 0, 0, 0, 0];
end

function HOME()
    global position home
    position = home;
    store();
end

%% 开始刻字过程

% Q
TL(180);
FD(150);
LB();
TR(180);
for i = 1:40
    FD(150*s);
    TL(10);
end
TL();
FD(35);
TL(180);
FD(70);

% J
TB();
RESET();
FD(100);
TL();
FD(175);
LB();
TL(180);
FD(125);
for i = 1:15
    FD(90*s);    
    TR(10);
end

% S
TB();
RESET();
FD(100);
FD(100);
TL();
FD(175);
TL();
LB();
for i = 1:18
    FD(90*s);
    TL(10);
end
for i = 1:24
    FD(90*s);
    TR(10);
end

TB();
RESET();
HOME();
% 1
TL(180);
FD(180);
TL();
FD(50);
TL();
LB();
TR();
FD(60);
BK(60);

% 1
TB();
TL();
FD(70);
LB();
TR();
FD(60);
BK(60);

% 4
TB();
TL();
FD(70);
LB();
TR();
FD(60);
BK(30);
TR();
FD(30);
TR();
FD(30);

% 9
TB();
TR();
FD(100);
LB();
TR(180);
FD(30);
TL();
FD(30);
TL();
FD(30);
TL();
FD(30);
TL(180);
FD(60);
TR();
FD(30);

% 复位
TB(); RESET(); HOME();

%% 将文件进行保存

fname = 'log.csv';
T = array2table(positions, 'VariableNames', title);
if exist(fname, 'file')
    delete(fname);
end
writetable(T, fname, 'WriteVariableNames', false);
disp('数据已保存到 log.csv');