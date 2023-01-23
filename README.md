## Docker Ros KHI-Robot 
# NEDO特別講座 
NEDO特別講座で扱う川崎重工株式会社公開のパッケージkhi-robotを動かす為の環境づくりを提供致します

# 環境構築 
実機のロボットを扱う際に計算機にリアルタイムカーネルの環境を構築する必要があります   
(※リアルタイムカーネルの構築は[Realtime.md](docs/Realtime.md)に詳細な記載をしています)  
リアルタイムカーネルの構築後にDockerエンジンのインストールを行い  
khi-robotのパッケージを実行するDockerのコンテナイメージをダウンロードします  

以下のスクリプトを実行することにより
```
. build.sh
```
リアルタイムカーネルのインストールから，Dockerエンジンのインストール, khi-robotを実行するコンテナイメージのダウンロードを行います

# 環境バージョン
講座内容に合わせる為にインストール時にDockerバージョンが指定されています  
Dockerがインストール済みになっている場合は一度削除され, 指定バージョンにインストールされます  
Dockerで指定しているバージョンは以下のバージョンとなっています
```
5:20.10.21~3-0~ubuntu-$(lsb_release -sc)
```
講座内で使用しているOSバージョンやRosバージョン下記の表通りとなっています

|項目|バージョン|
|:---|:---|
|OS|Ubuntu 20.04.5|
|REALTIME KERNEL|linux-5.4.221|
|ROS|Noetic|
|Docker|5:20.10.21~3-0~ubuntu|
|GPU|使用無し|

# Docker Engine Ros環境イメージの構築
Ros環境イメージを構築するDockerfileをビルドします  
ビルドの際にDocker Composeでymlファイルを経由してビルドします  
※ Dockerfileやdocker-compose.ymlが格納されているディレクトリは滅多に使用することがないので，隠しディレクトリとなっています
```
docker compose -f .docker/melodic/docker-compose.yml build
```
OR
```
docker compose -f .docker/noetic/docker-compose.yml build
```
しばらく経つとビルドが終わり, 下記の様に環境イメージが作成される.
```
docker images
```
|REPOSITORY|TAG|IMAGE ID|CREATED|SIZE|
|:---|:---|:---|:---|:---|
|skrjtech/khi_robot|ros_melodic|XXXXXXXX|X minutes ago|XXXXGB|
OR
|skrjtech/khi_robot|ros_noetic|XXXXXXXX|X minutes ago|XXXXGB|

## Docker Engine Ros環境イメージの起動
Ros環境イメージの起動には, Docker Composeでymlファイルを指定して仮想環境
を起動します. \
Docker Composeでの起動の際には, バックグランドオプションの'-d'を指定して起動します.
```
docker compose -f .docker/melodic/docker-compose.yml up -d
```
OR
```
docker compose -f .docker/noetic/docker-compose.yml up -d
```
仮想環境内でgazebo/rvizその他のGUI系アプリを使用していく為, ホスト側で仮想環境内のアプリを表示する許可を行います.
```
xhost +local:
```
上記の動作を行わないと下記の様な注意が表示されます \
`root@virtualenv:~$ rosrun rviz rviz` \
`QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-venv'` \
`Authorization required, but no authorization protocol specified` \
`qt.qpa.screen: QXcbConnection: Could not connect to display :0.0` \
`Could not connect to any X display.` 

### 環境が立ち上がったらgazeboとrvizの起動の確認
### いくつかのステップを踏んで動作をさしていく
tmuxの立ち上げ
```
tmux  
```
立ち上げ先でroscoreを起動
```
roscore 
```
[ Ctrl+b ]を押した後に[ c ]を押し新しいウィンドウを起動
新しいウィンドウを起動したrvizを起動する
```
rosrun rviz rviz
```
gazeboの起動もrvizの立ち上げと同様に行う
## 注意 GUIが正常に起動しない時の対処
計算機の環境, ホスト側がDockerの仮想環境に影響を与えている場合があることでしょう\
以下の例で
* GUIが起動するが一瞬で立ち上がって消える現象 \
    exportに`SVGA_VGPU10=0`を追加することで表示が可能になった \
    ホスト側とコンテナ側に追加
* 立ち上がる前にグラフィックの影響でエラーがでる現象 \
    ホスト側のグラフィックドライバーを消すことによって表示が可能になった \
    グラフィックドライバーを消すことでCPUのみの動作になってしまうことが難点 \
    只今模索中
* VMWare上でのGUIの表示でうまくいかない現象 \
    exportに`SVGA_VGPU10=0`を追加することで表示が可能になった \
    ホスト側とコンテナ側に追加
* など...
```
export SVGA_VGPU10=0
```

# 構築環境・途中構築環境のコミット
環境内で何かしらの構築が終わった場合(パッケージのインストール, など), 作業のやり直しを防げます
```
docker ps -a
```
|CONTAINER ID|IMAGE|COMMAND|CREATED|STATUS|PORTS|NAMES|
|---|---|---|---|---|---|---|
|XXXXXXXX|skrjtech/khi_robot:ros_melodic|"/bin/bash"|0 minutes ago|Up 0 minutes||ros_melodic_container|
OR
|XXXXXXXX|skrjtech/khi_robot:ros_noetic|"/bin/bash"|0 minutes ago|Up 0 minutes||ros_noetic_container|

$ docker commit CONTAINERID TAG
```
docker commit ros_melodic_container ros/khi_robot:<tag-name>
```
OR
```
docker commit ros_noetic_container ros/khi_robot:<tag-name>
```

# エラー発生・原因・対処について
