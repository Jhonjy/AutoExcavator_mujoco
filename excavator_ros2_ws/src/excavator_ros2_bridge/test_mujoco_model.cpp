// 独立MuJoCo测试程序：验证模型的equality connect约束是否正常工作
// 直接调用MuJoCo API，不经过ROS2
// 用法: ./test_mujoco_model [model_path]

#include <mujoco/mujoco.h>
#include <cstdio>
#include <cstring>
#include <string>

int main(int argc, char** argv) {
  std::string ws = EXCAVATOR_WS_ROOT;
  std::string model_path = ws + "/excavator_ros2_ws/src/excavator_ros2_bridge/config/excavator_control.xml";
  if (argc > 1) model_path = argv[1];

  // 加载插件
  std::string plugin_dir = ws + "/excavator_simulator_mujoco/build/bin/mujoco_plugin";
  mj_loadAllPluginLibraries(plugin_dir.c_str(), nullptr);

  // 加载模型
  char error[1024] = "";
  mjModel* m = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
  if (!m) {
    printf("模型加载失败: %s\n", error);
    return 1;
  }

  printf("模型加载成功\n");
  printf("  关节数: %d\n", m->njnt);
  printf("  执行器数: %d\n", m->nu);
  printf("  equality约束数: %d\n", m->neq);
  printf("  solver: iter=%d, tolerance=%f\n", m->opt.iterations, m->opt.tolerance);

  // 列出所有关节
  printf("\n关节列表:\n");
  for (int i = 0; i < m->njnt; ++i) {
    const char* name = mj_id2name(m, mjOBJ_JOINT, i);
    printf("  [%d] %s (type=%d, qposadr=%d, dofadr=%d)\n",
           i, name ? name : "(null)", m->jnt_type[i],
           m->jnt_qposadr[i], m->jnt_dofadr[i]);
  }

  // 列出所有执行器
  printf("\n执行器列表:\n");
  for (int i = 0; i < m->nu; ++i) {
    const char* name = mj_id2name(m, mjOBJ_ACTUATOR, i);
    printf("  [%d] %s (trntype=%d, target=%d)\n",
           i, name ? name : "(null)", m->actuator_trntype[i],
           m->actuator_trnid[2*i]);
  }

  // 列出所有equality约束
  printf("\nEquality约束:\n");
  for (int i = 0; i < m->neq; ++i) {
    const char* name1 = mj_id2name(m, mjOBJ_BODY, m->eq_obj1id[i]);
    const char* name2 = mj_id2name(m, mjOBJ_BODY, m->eq_obj2id[i]);
    printf("  [%d] type=%d body1='%s'(%d) body2='%s'(%d) solref=[%f,%f]\n",
           i, m->eq_type[i],
           name1 ? name1 : "(null)", m->eq_obj1id[i],
           name2 ? name2 : "(null)", m->eq_obj2id[i],
           m->eq_solref[2*i], m->eq_solref[2*i+1]);
  }

  // 创建仿真数据
  mjData* d = mj_makeData(m);
  mj_forward(m, d);

  // 记录初始状态
  printf("\n初始状态:\n");
  int boom_jnt = mj_name2id(m, mjOBJ_JOINT, "boom");
  int arm_jnt = mj_name2id(m, mjOBJ_JOINT, "arm");
  int bucket_jnt = mj_name2id(m, mjOBJ_JOINT, "bucket");
  int chassis_piston_jnt = mj_name2id(m, mjOBJ_JOINT, "chassis piston rod");
  int boom_piston_jnt = mj_name2id(m, mjOBJ_JOINT, "boom piston rod");
  int arm_piston_jnt = mj_name2id(m, mjOBJ_JOINT, "arm piston rod");

  printf("  chassis piston rod pos: %f\n", d->qpos[m->jnt_qposadr[chassis_piston_jnt]]);
  printf("  boom piston rod pos: %f\n", d->qpos[m->jnt_qposadr[boom_piston_jnt]]);
  printf("  arm piston rod pos: %f\n", d->qpos[m->jnt_qposadr[arm_piston_jnt]]);
  printf("  boom angle: %f rad (%.1f deg)\n",
         d->qpos[m->jnt_qposadr[boom_jnt]],
         d->qpos[m->jnt_qposadr[boom_jnt]] * 180.0 / 3.14159);
  printf("  arm angle: %f rad (%.1f deg)\n",
         d->qpos[m->jnt_qposadr[arm_jnt]],
         d->qpos[m->jnt_qposadr[arm_jnt]] * 180.0 / 3.14159);
  printf("  bucket angle: %f rad (%.1f deg)\n",
         d->qpos[m->jnt_qposadr[bucket_jnt]],
         d->qpos[m->jnt_qposadr[bucket_jnt]] * 180.0 / 3.14159);

  // 测试：设置Boom执行器控制值，运行仿真
  int boom_act = mj_name2id(m, mjOBJ_ACTUATOR, "Boom");
  int arm_act = mj_name2id(m, mjOBJ_ACTUATOR, "Arm");
  printf("\n测试: 设置Boom ctrl=0.3, 仿真5000步 (10秒)...\n");
  d->ctrl[boom_act] = 0.3;

  for (int step = 0; step < 5000; ++step) {
    mj_step(m, d);
    if (step % 1000 == 0) {
      printf("  step=%d time=%.3f piston=%.4f boom=%.4f(%.1f°) arm=%.4f(%.1f°) bucket=%.4f(%.1f°)\n",
             step, d->time,
             d->qpos[m->jnt_qposadr[chassis_piston_jnt]],
             d->qpos[m->jnt_qposadr[boom_jnt]],
             d->qpos[m->jnt_qposadr[boom_jnt]] * 180.0 / 3.14159,
             d->qpos[m->jnt_qposadr[arm_jnt]],
             d->qpos[m->jnt_qposadr[arm_jnt]] * 180.0 / 3.14159,
             d->qpos[m->jnt_qposadr[bucket_jnt]],
             d->qpos[m->jnt_qposadr[bucket_jnt]] * 180.0 / 3.14159);
    }
  }

  // 测试Arm
  printf("\n测试: 设置Arm ctrl=0.3, 仿真5000步...\n");
  d->ctrl[boom_act] = 0.0;
  d->ctrl[arm_act] = 0.3;

  for (int step = 0; step < 5000; ++step) {
    mj_step(m, d);
    if (step % 1000 == 0) {
      printf("  step=%d time=%.3f arm_piston=%.4f arm=%.4f(%.1f°)\n",
             step, d->time,
             d->qpos[m->jnt_qposadr[arm_piston_jnt]],
             d->qpos[m->jnt_qposadr[arm_jnt]],
             d->qpos[m->jnt_qposadr[arm_jnt]] * 180.0 / 3.14159);
    }
  }

  printf("\n完成\n");
  mj_deleteData(d);
  mj_deleteModel(m);
  return 0;
}
