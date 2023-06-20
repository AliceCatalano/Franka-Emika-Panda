% UDP Connection with Franka Interface - DO NOT CHANGE
hudps = dsp.UDPSender('RemoteIPPort',1500);
hudps.RemoteIPAddress = '10.211.55.12';

a = [0 0 0 0 0 0 0]';

for i = 1:inf
        posMsg = [a;a];
        step(hudps,posMsg)
        pause(1) 
end