# 单个缺陷检测结果
# 包含2D检测信息 + 3D定位信息 + 置信度

std_msgs/Header header   # 时间戳和frame_id

# 缺陷类别信息
int32 class_id            # 缺陷类别 0-5
string class_name         # 缺陷名称 (craze/corrosion/surface_injure/thunderstrike/crack/hide_craze)
float32 confidence        # 检测置信度 [0, 1]

# 2D检测框 (像素坐标)
float32 bbox_x_min        # 边界框左上x
float32 bbox_y_min        # 边界框左上y
float32 bbox_x_max        # 边界框右下x
float32 bbox_y_max        # 边界框右下y

# 3D世界坐标 (如果有深度/点云)
geometry_msgs/Point position_3d  # 缺陷在世界坐标系中的3D位置
bool has_3d              # 是否有3D定位结果
