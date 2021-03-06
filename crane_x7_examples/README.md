# crane_x7_example

このパッケージは、オリジナルである<https://github.com/rt-net/crane_x7_ros/tree/master/crane_x7_examples>を千葉工業大学未来ロボティクス学科の講義内グループ2班が変更を加えたものです。
<br>
このパッケージにより「crane_x7に筆を使って、文字を書かせる」ことができます。

このシステムでは「密」を書くことができます。

## システムの起動方法

CRANE-X7の制御信号ケーブルを制御用パソコンへ接続します。
Terminalを開き、`crane_x7_bringup`の`demo.launch`を起動します。
このlaunchファイルには次のオプションが用意されています。

- fake_execution (default: true)

実機を使用する/使用しない

### シミュレータを使う場合 

実機無しで動作を確認する場合、
制御信号ケーブルを接続しない状態で次のコマンドを実行します。

```sh
$ roslaunch crane_x7_bringup demo.launch fake_execution:=true
```

### 実機を使う場合

・実機起動

実機で動作を確認する場合、
制御信号ケーブルを接続した状態で次のコマンドを実行します。

```sh
$ roslaunch crane_x7_bringup demo.launch fake_execution:=false
```
・プログラム起動

まず次のディレクトリに移動します。

```sh
$ cd ~/catkin_ws/src/crane_x7_ros_modified_by_group2/crane_x7_examples/scripts/Practice
```

RealSenseを起動します。

```sh
$ roslaunch realsense2_camera rs_camera.launch
```

紙と水入れを設置した後,次のコマンドを実行します。

＊水入れは赤色のものを使用してください。それ以外は認識できません。

```sh
$ rosrun crane_x7_examples opencv.py
$ rosrun crane_x7_examples JSub_mituver.py
```

### gazeboを使う場合

・gazebo起動

次のコマンドで起動します。実機との接続やcrane_x7_bringupの実行は必要ありません。

```sh
$ roslaunch crane_x7_gazebo crane_x7_with_table.launch
```

・プログラム起動

次のコマンドでプログラムのディレクトリに移動します。

```sh
$ cd ~/catkin_ws/src/crane_x7_ros_modified_by_group2/crane_x7_examples/scripts/Practice 
```

次のコマンドでプログラムを実行します。

RealSenceを使用する場合

```sh
$ roslaunch realsense2_camera rs_camera.launch
$ rosrun crane_x7_examples opencv.py
$ rosrun crane_x7_examples Jsub_mituver.py
```
RealSenceを使用しない場合(決められたの位置に移動させたい場合)

```sh
$ rosrun crane_x7_examples JPub.py
$ rosrun crane_x7_examples JSub_mituver.py
```
