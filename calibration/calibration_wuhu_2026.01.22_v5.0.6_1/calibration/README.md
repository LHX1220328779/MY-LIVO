# calibration_json

calibration json files
##################################################################
version:calibration_20240401_v4.0_pre01
note:base
##################################################################
version:calibration_20240422_v4.0_pre05
note:
9724	配置	增加AEB跨行的开关
9528	配置	vehicle and control json update 
9533	配置	vehicle and control json update 
6967	配置	vehicle and control json update 
7079	配置	修改芜湖23号车去灰尘参数 
10076	配置	V4.0 perception/task#10076 wuhu26 perception json 
10096	配置	RTH136 control and vehicle paramter calibration mo
##################################################################
version:calibration_20240521_v4.1_pre03
note:
TASK	10175	配置	新增4.1 driverlidar callback fsm监测
TASK	10237	配置	wuhu truck15 perception json 
##################################################################
version:calibration_20240530_v4.1_pre05
note:
TASK	10258	配置	add upload gridmap ip and port
BUG	7683	配置	【2024-05-24】【ConchPilot4.1_2024.05.22_V4.1_pre03】【实车功能测试】【11号车】【8:50】频繁感知减速
TASK	10175	配置	[C]新增故障逻辑代码开发_Localization
##################################################################
version:calibration_20240603_v4.1_pre06
note:
TASK	10356	配置	 芜湖5号车由于右激光雷达损坏 激光雷达标定

##################################################################
version:calibration_20240607_v4.1_pre08
note:
BUG	7922	配置	调低芜湖12号车后向去灰尘参数
BUG	7590	配置	修改激光雷达目标检测置信度阈值
TASK	10392	配置	芜湖12号车传感器标定
TASK	10390	配置	【C】戴世组合导航数据改为602 CAN ID的海拔高且融合定位补偿差值为椭球高
##################################################################
version:calibration_20240607_v4.1_pre08
note:
TASK	10395	配置	3#更换为华测610组合导航，localization离线标定激光到地图参数适配修改 
##################################################################
version:calibration_20240609_v4.1_pre10
note:
TASK	10396	配置	wuhu 1 lidar calibration json update with back lidar position change on 2024.6.8 
##################################################################
version:calibration_20240612_v4.1_rel
note:
TASK	10435	配置	【D】芜湖卸料口地图优化以及矿车激光到3地图离线标定
BUG	8046	配置	【2024-06-11】【ConchPilot4.1_2024.06.09_V4.1_pre10】【全流程编组测试】【1号车】【19：12】80路口下山频繁感知减速
BUG	8038	配置	【2024-06-10】【ConchPilot4.1_2024.06.09_V4.1_pre10】【全流程编组测试】【3号车】【9:15】在卸料口融合定位零速漂移导致定位漂移
BUG	7992	配置	 【2024-06-07】【ConchPilot4.1_2024.06.03_V4.1_pre07】【横向控制策略优化】【横向误差大时，不需要下发大的期望转角跟踪路线】
##################################################################
version:calibration_20240614_v4.1_hotfix01
note:
BUG	8079	配置	【2024-06-13】【ConchPilot4.1_2024.06.12_V4.1_rel】【全流程编组测试】【17号车】【22:18】二级故障fusion接收lidardetection数据丢失，
TASK	10448	配置	芜湖1号车毫米波外参标定
TASK	10448	配置	 芜湖26号车鱼眼摄像头传感器标定
BUG	8096	配置	【2024-06-14】【ConchPilot4.1_2024.06.12_V4.1_rel】【模块测试】【motion】铲装区倒车,距离路径终点小于10m,正铲，应该不屏蔽障碍物
##################################################################
version:calibration_20240619_v4.1_hotfix04
note:
TASK	10480	配置	localization关闭融合定位 
TASK	10472	配置	【D】芜湖卸料口地图更新
TASK	10462	配置	芜湖16号车激光标定
##################################################################
version:calibration_20240621_v4.1_hotfix05-fix
note:
BUG	8046	代码	【2024-06-11】【ConchPilot4.1_2024.06.09_V4.1_pre10】【全流程编组测试】【1号车】【19：12】80路口下山频繁感知减速
##################################################################
version:calibration_20240622_v4.1_hotfix05-fix
note:
wuhu13 calibration json update due to aeb on 2024.6.22 & wuhu04 wuhu17 radar calibration json update for radar detection error
##################################################################
version:calibration_20240626_v4.1_hotfix06
note:
TASK	10570	配置	芜湖23车相机标定
##################################################################
version:calibration_20240703_v4.1_hotfix07
note:wuhu#5 calibration json update for demands of loca
##################################################################
version:calibration_20240705_v4.1_hotfix08
note:05# localization lidar-to-map update
##################################################################
version:calibration_20240706_v4.1_hotfix10
note:
TASK 10646 配置 wuhu#4 calibration json update for right radar position change on 2024.07.07
version:calibration_20240708_v4.1_hotfix11
note:wuhu#23 calibration json update for lidar point 
##################################################################
version:calibration_20240708_v4.1_hotfix12
note:wuhu 23 lidar-to-map update
##################################################################
version:calibration_20240710_v4.1_hotfix13
note:03 lidar-to-map更新 
##################################################################
version:calibration_20240716_v4.1_hotfix14
note:wuhu#04 radar calibration json update for percep 
##################################################################
version:calibration_20240717_v4.1_hotfix15
note:wuhu#04 lidar calibration json update
##################################################################
version:calibration_20240720_v4.1_hotfix16
note:wuhu#13 lidar calibration json update
##################################################################
version:calibration_20240726_v4.1_hotfix17
note:wuhu#11 camera calibration json update
##################################################################
version:calibration_20240728_v4.1_hotfix18
note:wuhu 12 calibration json update due to aeb on 2024.07.27 & 12# lidar-to-map标定
##################################################################
version:calibration_20240802_v4.1_hotfix19
note:hesai port update and lidar calibration json updat & wuhu 18 calibration json update 08 01
##################################################################
version:calibration_20240808_v4.1_hotfix19
note:wuhu #01 #18 camera calibration json update
##################################################################
version:calibration_20240812_v4.1_hotfix20
note:wuhu #01 #18 camera calibration json update
##################################################################
version:calibration_20240819_v4.1_hotfix21
note:wuhu wuhu#11 lidar calibration json update
##################################################################
version:calibration_20240819_v4.1_hotfix22
note:wuhu#1 lidar calibration json update for back lidar replace
##################################################################
version:calibration_20240827_v4.1_hotfix23
note:wuhu_11 fisheye camera internal and external parameters updated
##################################################################
version:calibration_20240829_v4.1_hotfix24
note:wuhu#12 lidar calibration update for back lidar replace
##################################################################
version:calibration_20240830_v4.1_hotfix25
note:18# update lidar-to-map 参数
##################################################################
version:calibration_20240901_v4.1_hotfix26
note:wuhu truck 16 lidar calibration update for back broken lidar arm change
##################################################################
version:calibration_20240902_v4.1_hotfix27
note:wuhu#1 fisheye camera internal and external parameters updated
##################################################################
version:calibration_20240903_v4.1_hotfix28
note:wuhu4# perception calibration json update for front lidar change on 9 3 & 11# 18# camera update
##################################################################
version:calibration_20240905_v4.1_hotfix29
note:wuhu#02 sensor calibration json update 09 05
##################################################################
version:calibration_20240906_v4.1_hotfix30
note:芜湖02# lidar-to-map参数更新
##################################################################
version:calibration_20240909_v4.1_hotfix31
note:wuhu#05#12#16 fisheye camera parameters update 09 08
##################################################################
version:calibration_20240923_v4.1_hotfix32
note:wuhu#17#13#03 fisheye camera external and internal parameters update
##################################################################
version:calibration_20240926_v4.1_hotfix33
note:wuhu13 lidar calibration json update
##################################################################
version:calibration_20240927_v4.1_hotfix34
note:wuhu#15 lidar and fisheye camera parameters 
##################################################################
version:calibration_20240929_v4.1_hotfix35
note:wuhu#05 sensors external parameters translate 10cm
##################################################################
version:calibration_20241008_v4.1_hotfix36
note:wuhu#18 sensors external parameters update
##################################################################
version:calibration_20241029_v4.1_hotfix37
note:wuhu#17 camera calibration update
##################################################################
version:calibration_20241031_v4.1_hotfix38
note:wuhu#11 back lidar calibration update,wuhu#05 camera parameters update
##################################################################
version:calibration_20241029_v4.1.1_pre06
note:update for v4.1.1_pre06
##################################################################
version:calibration_20241106_v4.1_hotfix39
note:wuhu#12 calibration update
##################################################################
version:calibration_20241110_v4.1_hotfix40
note:wuhu#18 #17 lidar external parameters update
##################################################################
version:calibration_20241122_v4.1_hotfix41
note:wuhu#13 lidar calibration json update for lidar arm change
##################################################################
version:calibration_20250115_v4.1.4_hotfix01
note:tr100e init
##################################################################
version:calibration_20250211_v4.1.4_hotfix02
note:27&28融合定位标定
##################################################################
version:calibration_20250218_v4.1.4_hotfix03
note:28#融合定位参数标定更新 
##################################################################
version:calibration_20250324_v4.1.4_hotfix03
note:27#参数标定更新 
##################################################################
version:calibration_20250331_v4.1.4_hotfix03
note:27#参数标定更新 