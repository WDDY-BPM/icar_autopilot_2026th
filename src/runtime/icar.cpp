#include "icar.hpp"

void Icar::callbackMouse(int event, int x, int y, int flags, void *userdata)
    {
        Icar *self = static_cast<Icar *>(userdata);
        if (self)
            self->handleMouse(event, x, y, flags);
    }

void Icar::handleMouse(int event, int x, int y, int flags)
    {
        double value;
        switch (event)
        {
        case cv::EVENT_MOUSEWHEEL: // 鼠标滑球
        {
            value = cv::getMouseWheelDelta(flags); // 获取滑球滚动值
            if (value > 0)
                show->index++;
            else if (value < 0)
                show->index--;

            if (show->index < 0)
                show->index = 0;
            if (show->index > show->frameMax)
                show->index = show->frameMax;
            break;
        }
        default:
            break;
        }
    }

Icar::Icar()
    {
        params = make_shared<Params>();                        // 初始化参数
        center = make_shared<Center>();                        // 控制中心处理类
        motion = make_shared<Motion>();                        // 运动控制器
        predeal = make_shared<Predeal>(params->config.binary); // 图像预处理类
        detection = make_shared<Detection>(params->config.model,
                                           params->config.score); // AI模型初始化

        // 初始化TCP通信客户端
        client = make_shared<Client>();
        if (!client->start())
        {
            throw std::runtime_error("[Error]: Socket init failed");
        }
        client->buzzerSound(client->BUZZER_OK); // 提示音效

        // 相机初始化
        // USB摄像头初始化
        frameCapture = make_shared<LatestFrameCapture>();
        if (params->config.debug)
            frameCapture->open(params->config.video);
        else
        {
            if (!frameCapture->open("/dev/video0", cv::CAP_V4L2))
                frameCapture->open("/dev/video0");
        }
        if (!frameCapture->isOpened())
        {
            throw std::runtime_error("[Error]: Can not open video device");
        }
        frameCapture->set(cv::CAP_PROP_FRAME_WIDTH, COLSCAMERA);  // 设置图像分辨率
        frameCapture->set(cv::CAP_PROP_FRAME_HEIGHT, ROWSCAMERA); // 设置图像分辨率
        frameCapture->set(cv::CAP_PROP_FPS, 30);                  // 设置帧率
        if (!params->config.debug)
            frameCapture->set(cv::CAP_PROP_BUFFERSIZE, 1);

        if (params->config.debug)
        {
            show = make_shared<Show>(4); // 调试UI初始化
            show->frameMax = frameCapture->get(cv::CAP_PROP_FRAME_COUNT) - 1;
            cv::createTrackbar("Frame", "ICAR", &show->index, show->frameMax, [](int, void *) {}); // 创建Opencv图像滑条控件
            cv::setMouseCallback("ICAR", &Icar::callbackMouse, this);
        }
        else
        {
            std::cout << "[Camera] Latest-frame capture enabled; backend="
                      << frameCapture->get(cv::CAP_PROP_BACKEND) << std::endl;
            frameCapture->start();
        }

        // FSM有限状态机初始化
        fsmFactory.busy = make_shared<FsmBusy>(params);         // 避障控制实例化
        fsmFactory.park = make_shared<FsmPark>(params);         // 停车场控制实例化
        fsmFactory.stop = make_shared<FsmStop>(params);         // 斑马线停车控制实例化
        fsmFactory.cross = make_shared<FsmCross>(params);       // 斑马线停车控制实例化
        fsmFactory.fork = make_shared<FsmFork>(params);         // 停车场岔路控制实例化
        fsmFactory.slow = make_shared<FsmSlow>(params);         // 慢行区控制实例化
        fsmFactory.obstacle = make_shared<FsmObstacle>(params); // 全局障碍物检测实例化
        fsmFactory.yfork = make_shared<FsmYfork>(params);       // Y型岔路口控制实例化
        fsmFactory.station = make_shared<FsmStation>(params);   // 停靠站控制实例化

        // 手动控制线程初始化
        fsmFactory.manual = make_shared<ManualControlThread>();

        // 启动AI推理子线程
        loops = make_shared<Loops>("LoopAI", 1.f / 30.f, std::bind(&Icar::runModel, this));
        loops->start(); // RL开始推理

        // 启动手动控制线程
        fsmFactory.manual->start();

        printf("[OK]: Params initial succeed!!!\n");
    }

Icar::~Icar()
    {
        shuttingDown = true;
        cvImg.notify_all();
        if (loops)
            loops->shutdown();
        if (fsmFactory.manual)
            fsmFactory.manual->stop();
        if (frameCapture)
            frameCapture->stop();
    }
