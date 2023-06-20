function SlowdownToRealtime(deltat)
condition = true;
tic
while condition
    if (toc >= deltat)
        condition = false;
    end
end
tic