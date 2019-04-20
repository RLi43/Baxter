# 错误记录

## 消失的节点

如果你发现程序可以运行，但不能实现你要的功能，翻一翻节点列表时发现竟然没有这个节点。那么，你很可能有两个master节点，尝试在各个终端`echo $ROS_MASTER_URI`，如果不相同，那么将它们改成一致的`export ROS_MASTER=XXX`

## Char?unsigned int?

在创建一个包含char的消息后，发现发送char类型数据时出错。查找官网，发现char被废弃，作为uint8的代称，所以不能用char类型，可以考虑`ord('A')`