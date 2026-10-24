cmake_minimum_required(VERSION 3.5)
project(pm_config)

find_package(ament_cmake REQUIRED)

# 設定専用パッケージ。既存launchが使うshare/pm_config/configの場所は維持する。
# workspaceのcolcon_defaults.yamlでsymlink-installを既定にするため、既存YAMLの
# 内容変更は再ビルドせず参照先へ反映される。nodeへの反映には再起動が必要。
# サブディレクトリも同じ処理で配置する。ファイル追加・削除時には再ビルドする。
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
