function Generate_txt( data, Filename)

[rows,cols] = size(data);
fid = fopen( [Filename '.txt'], 'wt' );

%% Generate TXT
for j = 1:cols
    for i = 1:rows
        fprintf(fid,'%f\t',data(i,j));
    end
    fprintf(fid,'\n');
end

fclose(fid);
end