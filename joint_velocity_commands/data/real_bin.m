text = strsplit(fileread('q_traj.txt'), {'\r', '\n'});  %read file and split into lines
isvalid = ~cellfun(@(t) any(t == 'x') | numel(t) < 32, text);  %line scontaining 'x' or shorter than 32 bits are not valid
unsigned = bin2dec(text(isvalid));  %convert valid lines from binary to decimal. bin2dec always assume unsigned
signed = typecast(uint32(unsigned), 'int32')   %convert from unsigned to signed