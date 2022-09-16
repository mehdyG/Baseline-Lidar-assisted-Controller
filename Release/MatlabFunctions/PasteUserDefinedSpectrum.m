% Created: 
% Feng Guo on 21-Mar-2020
% (c) Hochschule Flensburg WETI 
% ----------------------------------


function PasteUserDefinedSpectrum(NewFileName,PSD)

fileID = fopen(NewFileName,'a');
%fmt = '%12.8f %12.8f %12.8f %12.8f \n';
fmt = '%e %e %e %e \n';
%fmt = '%12.8f %12.8f %12.8f %12.8f \n';
fprintf(fileID,fmt,PSD');
fclose(fileID);
end

