// 模拟viewer的逻辑：设置活塞qpos，调用mj_forward，检查boom是否跟着动
#include <mujoco/mujoco.h>
#include <cstdio>

int main() {
  const char* model_path = "/home/ubuntu2204/mujoco_develop/excavator_ros2_ws/src/excavator_ros2_bridge/config/excavator_control.xml";
  const char* plugin_dir = "/home/ubuntu2204/mujoco_develop/excavator_simulator_mujoco/build/bin/mujoco_plugin";

  mj_loadAllPluginLibraries(plugin_dir, nullptr);

  char error[1024] = "";
  mjModel* m = mj_loadXML(model_path, nullptr, error, sizeof(error));
  if (!m) { printf("加载失败: %s\n", error); return 1; }

  mjData* d = mj_makeData(m);
  mj_forward(m, d);

  // 获取关节ID
  int chassis_jnt = mj_name2id(m, mjOBJ_JOINT, "chassis");
  int chassis_piston_jnt = mj_name2id(m, mjOBJ_JOINT, "chassis piston rod");
  int boom_piston_jnt = mj_name2id(m, mjOBJ_JOINT, "boom piston rod");
  int arm_piston_jnt = mj_name2id(m, mjOBJ_JOINT, "arm piston rod");
  int boom_jnt = mj_name2id(m, mjOBJ_JOINT, "boom");
  int arm_jnt = mj_name2id(m, mjOBJ_JOINT, "arm");
  int bucket_jnt = mj_name2id(m, mjOBJ_JOINT, "bucket");

  printf("=== 模拟viewer逻辑 ===\n");
  printf("初始状态:\n");
  printf("  boom piston: %.4f, boom angle: %.4f (%.1f°)\n",
         d->qpos[m->jnt_qposadr[boom_piston_jnt]],
         d->qpos[m->jnt_qposadr[boom_jnt]],
         d->qpos[m->jnt_qposadr[boom_jnt]] * 180.0 / 3.14159);

  // 模拟viewer: 只设置活塞杆qpos，然后mj_forward
  // 先用mj_step把活塞推进一段距离
  d->ctrl[mj_name2id(m, mjOBJ_ACTUATOR, "Boom")] = 0.3;
  for (int i = 0; i < 500; ++i) mj_step(m, d);

  double piston_after_step = d->qpos[m->jnt_qposadr[chassis_piston_jnt]];
  double boom_after_step = d->qpos[m->jnt_qposadr[boom_jnt]];
  printf("\n经过500步mj_step后:\n");
  printf("  piston: %.4f, boom: %.4f (%.1f°)\n",
         piston_after_step, boom_after_step, boom_after_step * 180.0 / 3.14159);

  // 现在模拟viewer: 重置所有qpos到初始值，只设置活塞位置，调用mj_forward
  mj_resetData(m, d);
  d->qpos[m->jnt_qposadr[chassis_piston_jnt]] = piston_after_step;
  mj_forward(m, d);

  double boom_after_forward = d->qpos[m->jnt_qposadr[boom_jnt]];
  printf("\nviewer模拟（重置后只设piston qpos，mj_forward）:\n");
  printf("  piston: %.4f, boom: %.4f (%.1f°)\n",
         d->qpos[m->jnt_qposadr[chassis_piston_jnt]],
         boom_after_forward, boom_after_forward * 180.0 / 3.14159);

  // 再试一次，这次不重置，直接设置piston并mj_forward
  mj_resetData(m, d);
  mj_forward(m, d);  // 初始化
  d->qpos[m->jnt_qposadr[chassis_piston_jnt]] = piston_after_step;
  mj_forward(m, d);

  printf("\nviewer模拟2（不重置，直接设piston qpos，mj_forward）:\n");
  printf("  piston: %.4f, boom: %.4f (%.1f°)\n",
         d->qpos[m->jnt_qposadr[chassis_piston_jnt]],
         d->qpos[m->jnt_qposadr[boom_jnt]],
         d->qpos[m->jnt_qposadr[boom_jnt]] * 180.0 / 3.14159);

  // 再试多次mj_forward
  for (int i = 0; i < 10; ++i) mj_forward(m, d);
  printf("\n10次mj_forward后:\n");
  printf("  piston: %.4f, boom: %.4f (%.1f°)\n",
         d->qpos[m->jnt_qposadr[chassis_piston_jnt]],
         d->qpos[m->jnt_qposadr[boom_jnt]],
         d->qpos[m->jnt_qposadr[boom_jnt]] * 180.0 / 3.14159);

  mj_deleteData(d);
  mj_deleteModel(m);
  return 0;
}
