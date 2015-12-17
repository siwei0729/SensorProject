function readData(obj)


rec = fread(obj, [1 1], 'uint8'); % ????1???
if rec ~= ','
    s = s+rec;
else
    switch count
        case 0
            numX = str2double(s);
            s = '';
            count = count + 1;
        case 1
            numY = str2double(s);
            s = '';
            count = count +1;
        case 2
            numZ = str2double(s);
            s = '';
            count = 0;
    end
end
disp(numX, numY, numZ);
end