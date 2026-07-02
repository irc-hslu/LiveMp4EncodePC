#build project 
cmake --build D:\LiveStream\LiveMp4EncodePC\build --config Release --target vpcclive_streamer

#Run web relay server in new terminal
cd web\server
npm run relay

#Run http server in new terminal
cd web\server
npm run serve

#Run c++ capture to V-PCC encoding app
cd build\Release
vpcclive_streamer.exe --address 127.0.0.1 --port 8890 --preview-port 8891 --fps 30 --encode-fps 3 --encode-points 20000 --geo-bits 9 --preview-points 20000 --threads 4