% Scan ROSCO input text file to get the specific variable
% e.g. input
% VariableName     = 'WE_FOPoles_v';
% ROSCOInFileName  = 'ROSCO2.IN';
function   VlueFound = GetParametersFromROSCO(ROSCOInFileName,VariableName)

fid             = fopen(ROSCOInFileName);

while ~feof(fid)
   s            = fgetl(fid);
   if contains(s,VariableName )
       
      s_temp    = extractBefore(s,VariableName);
      s_temp    = extractBefore(s_temp,'!');
      VlueFound = cell2mat(textscan(s_temp,'%f'));
      break
   end
   
       
end

fclose(fid);