uaClient = opcua('192.168.0.1', 4840);
connect(uaClient)

convEnd = opcuanode(3, '"workpieceAtConvEnd"', uaClient);
stackStat = opcuanode(3, '"workpieceAvailable"', uaClient);

t_start = tic;
log_data = []; %#ok<*AGROW>
temp = 0;
while temp < 180
    read1 = readValue(uaClient, stackStat);
    read2 = readValue(uaClient, convEnd);
    temp = toc(t_start);
    log_data = [log_data; [temp, read1, read2]];
end

