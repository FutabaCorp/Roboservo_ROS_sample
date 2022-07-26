# Roboservo_sample

これはRoboservoにros_canopenを使用してros_controlから動作指令を送るサンプルコードです。  
<br>
* ros_controlのコントローラに発行された指令をros_canopenによってCANopen指令へ変換し、Roboservoを制御します。  
* パラメータサーバへサーボモータのURDFを登録します。  
* ros_canopenへEDS(DCF)ファイルを登録し、CANopen通信設定を行い、canopen_motor_nodeを起動します。  
* ros_controlへコントローラ(位置指令、速度指令、トルク指令から選択)を設定し、controller_managerを起動します。  
* CANopenドライバを初期化し、CANopen通信を確立します。
* 通信確立後、コントローラのトピックへ指令を発行しRoboservoの位置制御を行う事が可能になります。  
* 各制御の指令パラメータの一部(目標回転数、加減速度等)はDynamic reconfigureから変更する事が可能です。
<br>
ros_canopenおよびros_controlについては下記をご参照ください。

* ros_canopen:　http://wiki.ros.org/ros_canopen
* ros_control:　http://wiki.ros.org/ros_control

```
roboservo_sample
├── cfg
│   ├── controller.yaml		: ros_controlコントローラ設定
│   ├── roboservo_param.cfg	: Dynamic reconfigureのパラメータ設定
│   └── setting.yaml		: CANopen通信設定
├── dcf
│   └── Futaba_Roboservo_Series.dcf	: CANopenオブジェクトディクショナリ
├── launch
│   └── setup.launch		: 各ノードの起動
├── src
│   └── roboservo_sample_node.cpp : Dynamic reconfigure設定
├── urdf
│   └── roboservo.urdf		: Roboservoを含むURDF
├── CMakeLists.txt
├── install_pkg.sh
├── package.xml
└── README.md
```
<br>


# Requirement

本サンプルコードでは下記のパッケージを使用します。  

* ros_canopen
* can_msgs
* canopen_402
* canopen_chain_node
* canopen_master
* canopen_motor_node
* socketcan_bridge
* socketcan_interface
* ros_control
* controller_interface
* controller_manager
* controller_manager_msgs
* hardware_interface
* joint_limits_interface
* transmission_interface
* dynamic_reconfigure
<br>


# Installation

"install_pkg.sh"を使用して必要なパッケージをインストールする事が可能です。  
<br>


# Usage

本サンプルコードをcatkin_wsなどのROSワークスペースへコピーし、必要なパッケージをインストールしてください。  
<br>

## 使用手順

1. ターミナルからCANインターフェイス（本サンプルコードでは"can0"）を有効にします
2. "setup.launch"を起動し、各ノードを起動します<br>
Roboservoから「カチッカチッ」とブレーキが外れる音がすれば起動完了です。
```
#Set CAN interface
sudo ip link set can0 up type can bitrate 1000000

#Start sample code
roslaunch roboservo_sample setup.launch
```

3. 別のターミナルを開き、コントローラのトピックへ動作指令を発行します  
ros_controlからros_canopenを通してRoboservoへ指令が送信されます。  
サンプルコードのデフォルトでは位置指令コントローラ(motor_position_controller)が選択されています。  
```
rostopic pub -1 /motor_position_controller/command std_msgs/Float64 "data: X" (X：指令値[rad])
```

4. パラメータの一部はDynamic reconfigureから変更が可能です。
* Profile_velocity　：　位置制御での目標回転数[rpm]
* Profile_acceleration　：　位置制御、速度制御での加速度[rpm]
* Profile_deceleration　：　位置制御、速度制御での減速度[rpm]
<br>

## 複数台の接続

複数台のRoboservoを接続する場合は、下記のファイルを変更する必要があります。  
詳細は個別の説明をご参照ください。
* controller.yaml
* setting.yaml
* roboservo_param.cfg
* setup.launch
<br>

## サンプルコード内ファイル説明
### controller.yaml
ros_controlで使用するコントローラを設定します。  
本サンプルではURDFへ記述した2つのジョイント"roboservo_1"、"roboservo_2"に対して位置制御コントローラを宣言しています。  

* **motor_position_controller**  
位置指令のコントローラになります。  
*required_drive_mode*によってCANopen CiA402の制御モードを指定しています。  
"Profile position mode"は加減速度を考慮した位置制御となります。  
目標回転速度、加速度、減速度はDynamic configureからの調整が可能です。  
```
type: position_controllers/JointPositionController
required_drive_mode: 1 (Profile position mode)
```

### roboservo_param.cfg
Dynamic reconfigureで使用するパラメータを設定します。  
調整可能な項目は目標回転速度、加速度、減速度となります。  

* **Joint_name**  
調整する対象のジョイントを選択します。  
2台目以降は"joints_enum"へ追加したいジョイントを記載してください。  
本サンプルでは2台のジョイント("roboservo_1"、"roboservo_2")から選べる用にしています。  

* **Profile_velocity**  
位置制御における目標回転数を設定します。  
下記の第5引数にて初期値、第6引数にて最小値、第7引数にて最大値を設定できます。  
```
(例)
gen.add("Profile_velocity", double_t, 0, "Profile velocity", 10.0(初期値), 0.1(最小値), 32.0(最大値))
```

* **Profile_acceleration**  
位置指令、速度指令における加速度を設定します。  
初期値、最小値、最大値については"Profile_velocity"と同じ設定方法です。  

* **Profile_deceleration**  
位置指令、速度指令における減速度を設定します。  
初期値、最小値、最大値については"Profile_velocity"と同じ設定方法です。  
<br>

### setting.yaml
CANopenの通信設定を行います。  

* **CANopen Bus layer**  
使用するCANインターフェイス、SYNC周期、ハートビートプロデューサの設定を行います。  
CANインターフェイスを変更する場合は、"bus"を書き換えてください。

* **ROS layer**  
パブリッシュするパラメータ、ros_control指令値からCANopen指令値への変換式を設定します。  
"publish"	: 任意のCANopenオブジェクトをトピックへパブリッシュする事ができます。  
"pos_to_device"	: 位置指令における指令値を変換します。 inc = "1周分のinc" * rint(rad2deg( pos )) / "360度"  
"vel_to_device"	: 速度指令における指令値を変換します。 (調整中)
"eff_to_device"	: トルク指令における指令値を変換します。 (調整中)

* **Node layer**  
CANopenノードの設定をします。  
対象のジョイント名、CANopenノードID、EDS(DCF)ファイルパスを指定してください。  
3台目以降はここに追記してください。  
<br>

### Futaba_Roboservo_Series.dcf
RoboservoのCANopenオブジェクトディクショナリです。  
ros_canopen仕様によりオブジェクトの一部はPDOマッピングに指定する必要があります。  
```
位置指令(Profile position mode)			: 607Ah Target position
速度指令(Profile velocity mode)			: 60FFh Target velocity
トルク指令(Cyclic synchronous torque mode)	: 6071h Target torque 
```
<br>

### setup.launch
ros_controlおよびros_canopen、dynamic_reconfigureの登録および起動を行います。  
なお、起動するコントローラを変更/追加する場合は下記部分を変更してください。  
```
<!-- set controller -->  
<rosparam command="load" file="$(find roboservo_sample)/cfg/controller.yaml" />  
<node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false" output="screen"   
  args="state_controller "任意のコントローラを指定" "/>  
```
<br>

### roboservo_sample_node.cpp
Dynamic reconfigureから変更したパラメータをRoboservoへ送信するノードです。
ノードの内容は下記になります。
1. Dynamic reconfigureの変更を取得するコールバックを設定(main関数)
2. 登録したパラメータの現在地を取得(main関数)
3. Dynamic reconfigureGUIから変更されると、コールバックして値をRoboservoへ送信(callback関数)
<br>

### roboservo.urdf
Roboservoを含むURDFを設定します。
実装する全てのRoboservoをジョイントとしてURDFに設定する必要があります。
<br>


# Note

現状、対応するコントローラは位置指令のみとなります。
速度指令、トルク指令につきましても引き続き開発中です。
<br>


# License

本サンプルコードはMITライセンスが適用されます。