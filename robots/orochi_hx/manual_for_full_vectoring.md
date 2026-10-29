orochi_hx 全駆動モデルの仕様書
==

* dragonと同様に2自由度の関節自由度, 2自由度のジンバル自由度とする。このとき、関節自由度はrollとyaw、ジンバル自由度はrollとpitchである。
* dragonのlinkに当たる部分はorochi_hxのfemaleであるとする。
* 各関節の変数を次のように定める
    * male_rollに起因するroll角: phi^m
    * gimbal_pitchに起因するpitch角: th^g
    * female_rollに起因するroll角: phi^f
    * joint_yawに起因するyaw角: psi^j
* 2自由度joint, 2自由度ジンバルのorochi_hxの制御に用いるロボットモデルを次のように定める
    * 関節自由度のrollの部分: phi_j
    * 関節自由度のyawの部分 : psi_j
    * ジンバル自由度のrollの部分: phi_g
    * ジンバル自由度のpitchの部分 : th_g
* このとき、phi_j, psi_j, phi_g, th_gはdragonと同様に求めた後にphi^m, th^g, phi^f, psi^jに分配するとすると、
    * psi^j = psi_j
    * th^g = th_g
    * phi^f = phi_g
    * phi^m = phi_j - phi_g
* となる。
* Servo.yamlには値の更新にジンバル自由度の角度を使う、th^g, phi^f, phi^mをgimbalsに、psi^jはjointに分類する。
* 命名は次のようにする。
    * orochi_hx_full_vectoring_controller.cpp
    * orochi_hx_full_vectoring_robot_model.cpp
    * quad/full_vectoring_202610/...
    * こんな感じで、必要なファイルを別途作成するイメージです。
