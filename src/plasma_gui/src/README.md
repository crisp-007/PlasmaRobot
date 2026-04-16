## 命名规范总结
### 1. 类名命名
- 使用PascalCase（首字母大写的驼峰）
- 注意：那三个opengl的类，尾部的gl不参与分割
- 例如： ApplicationManager , ProcessConfig , ProcessType
### 2. 函数/方法命名
- 使用PascalCase（首字母大写的驼峰）
- 例如： StartProcess() , StopProcess() , ConfigureProcess()
### 3. 成员变量/普通变量/常量命名
- 使用小写字母，用下划线分隔
- 例如： config_set , process_manager , display_name , auto_start