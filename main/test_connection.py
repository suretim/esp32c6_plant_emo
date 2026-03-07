
## 8. tools/test_connection.py

```python
#!/usr/bin/env python3
"""
ESP32连接测试工具
"""

import requests
import sys
import time

def test_connection(ip):
    """测试与ESP32的连接"""
    endpoints = [
        ("健康检查", f"http://{ip}/health"),
        ("缓冲区信息", f"http://{ip}/buffer_info"),
        ("最新数据", f"http://{ip}/latest"),
    ]
    
    print(f"测试连接 ESP32 @ {ip}")
    print("=" * 50)
    
    for name, url in endpoints:
        try:
            resp = requests.get(url, timeout=2)
            if resp.status_code == 200:
                print(f"✅ {name}: 成功")
                print(f"   响应: {resp.json()}")
            else:
                print(f"❌ {name}: HTTP {resp.status_code}")
        except Exception as e:
            print(f"❌ {name}: {e}")
        print()

def monitor_live(ip, interval=1):
    """实时监控数据"""
    print(f"开始实时监控 @ {ip}")
    print("=" * 50)
    
    try:
        while True:
            resp = requests.get(f"http://{ip}/latest", timeout=1)
            if resp.status_code == 200:
                data = resp.json()
                print(f"\r索引: {data['cursor']}, "
                      f"CI: {data['physio'][0]:.3f}, "
                      f"PAR: {data['physio'][1]:.3f}", end="")
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\n监控停止")
    except Exception as e:
        print(f"\n错误: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python test_connection.py <ESP32_IP> [monitor]")
        sys.exit(1)
    
    ip = sys.argv[1]
    
    if len(sys.argv) > 2 and sys.argv[2] == "monitor":
        monitor_live(ip)
    else:
        test_connection(ip)