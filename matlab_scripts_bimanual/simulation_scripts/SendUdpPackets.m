function [] = SendUdpPackets(pandaArm,uArm)
    

    packetArm = pandaArm.q';
    fwrite(uArm,packetArm,'uint8');
    

end

