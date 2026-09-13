# 许可与来源

自编业务代码及文档采用根目录MIT许可证，版权归Nway-J。该许可不替代第三方条款。

ECU-A/FreeRtos和ECU-B/FreeRtos中的内核、端口和heap实现来自FreeRTOS-Kernel V11.1.0，MIT许可，保留源码版权及LICENSE.md。具体来源与差异见 [FreeRTOS配置说明](docs/FreeRTOS配置说明.md)。应用配置基于同版本官方模板，保留其版权头。

ST标准外设库、CMSIS、启动文件以及配置模板的完整原始许可未在当前本地副本中恢复，因此这些依赖不纳入公开Git文件。请从供应方取得对应包并遵守其条款，使用 tools/install-dependencies.ps1 恢复；SHA256清单用于复现，不能替代授权。根MIT不授予这些组件的权利。

旧私有仓库含不同版本ST依赖历史，因此本公开仓库采用经过整理的源码快照，旧仓库保持私有。
