function error = check2PI(r_e)
for i = 1:size(r_e)
    if  r_e(i) < -3.14
        r_e(i) = r_e(i) + 6.28;
    end
    if r_e(i) > 3.14
        r_e(i) = r_e(i) - 6.28;
    end
end

error = r_e;

end