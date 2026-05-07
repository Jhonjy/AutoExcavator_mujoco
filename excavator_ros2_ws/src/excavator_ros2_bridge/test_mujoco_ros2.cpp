// 模拟ROS2硬件接口的MuJoCo调用流程，排查问题
#include <mujoco/mujoco.h>
#include <cstdio>
#include <string>

int main() {
  std::string ws = EXCAVATOR_WS_ROOT;
  std::string model_path = ws + "/excavator_ros2_ws/src/excavator_ros2_bridge/config/excavator_control.xml";
  std::string plugin_dir = ws + "/excavator_simulator_mujoco/build/bin/mujoco_plugin";

  // 加载插件
  mj_loadAllPluginLibraries(plugin_dir.c_str(), nullptr);

  // 加载模型
  char error[1024] = "";
  mjModel* m = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
  if (!m) { printf("加载失败: %s\n", error); return 1; }

  mjData* d = mj_makeData(m);
  mj_forward(m, d);

  // 获取关节ID（与硬件接口相同的方式）
  int chassis_jnt = mj_name2id(m, mjOBJ_JOINT, "chassis");
  int chassis_piston_jnt = mj_name2id(m, mjOBJ_JOINT, "chassis piston rod");
  int boom_piston_jnt = mj_name2id(m, mjOBJ_JOINT, "boom piston rod");
  int arm_piston_jnt = mj_name2id(m, mjOBJ_JOINT, "arm piston rod");
  int boom_jnt = mj_name2id(m, mjOBJ_JOINT, "boom");
  int arm_jnt = mj_name2id(m, mjOBJ_JOINT, "arm");
  int bucket_jnt = mj_name2id(m, mjOBJ_JOINT, "bucket");

  // 获取执行器ID
  int rot_act = mj_name2id(m, mjOBJ_ACTUATOR, "Rotation");
  int boom_act = mj_name2id(m, mjOBJ_ACTUATOR, "Boom");
  int arm_act = mj_name2id(m, mjOBJ_ACTUATOR, "Arm");
  int bucket_act = mj_name2id(m, mjOBJ_ACTUATOR, "Bucket");

  printf("关节ID: chassis=%d, chassis_piston=%d, boom_piston=%d, arm_piston=%d\n",
         chassis_jnt, chassis_piston_jnt, boom_piston_jnt, arm_piston_jnt);
  printf("执行器ID: rot=%d, boom=%d, arm=%d, bucket=%d\n",
         rot_act, boom_act, arm_act, bucket_act);

  // 模拟ROS2的read/write循环
  // 设置Boom ctrl=0.3
  d->ctrl[boom_act] = 0.3;
  printf("\n设置Boom ctrl=0.3, 模拟ROS2 100Hz循环 (每步调用mj_step)...\n");

  for (int i = 0; i < 500; ++i) {  // 500步 = 5秒@100Hz
    mj_step(m, d);

    if (i % 100 == 0) {
      double piston = d->qpos[m->jnt_qposadr[chassis_piston_jnt]];
      double boom_angle = d->qpos[m->jnt_qposadr[boom_jnt]];
      printf("  i=%d time=%.3f piston=%.6f boom=%.6f(%.2f°) ctrl=%f\n",
             i, d->time, piston, boom_angle,
             boom_angle * 180.0 / 3.14159, d->ctrl[boom_act]);
    }
  }

  // 检查ctrl是否真的写入了
  printf("\n最终ctrl值: [%.3f, %.3f, %.3f, %.3f]\n",
         d->ctrl[rot_act], d->ctrl[boom_act], d->ctrl[arm_act], d->ctrl[bucket_act]);
  printf("最终piston位置: %.6f (初始0.340)\n",
         d->qpos[m->jnt_qposadr[chassis_piston_jnt]]);

  mj_deleteData(d);
  mj_deleteModel(m);
  return 0;
}
