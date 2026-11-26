## Implementation examples for publishing measurements

### python

```bash
python3 publish.py
```

### cpp

```bash
sudo apt install libcurl4-openssl-dev nlohmann-json3-dev
g++ -std=c++17 -Wall -Wextra -O2 publish.cpp -o publish -lcurl
./publish
```