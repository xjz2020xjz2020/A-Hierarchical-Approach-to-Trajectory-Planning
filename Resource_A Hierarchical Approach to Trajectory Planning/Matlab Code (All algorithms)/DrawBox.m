function DrawBox(vec,ind)
x = [vec(1), vec(3), vec(5), vec(7), vec(1)];
y = [vec(2), vec(4), vec(6), vec(8), vec(2)];
if (ind == 1)
    plot(x,y,'r');
else
    plot(x,y,'k');
end
end