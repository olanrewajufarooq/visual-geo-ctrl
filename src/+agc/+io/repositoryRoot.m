function root = repositoryRoot()
%REPOSITORYROOT Absolute repository root, independent of MATLAB pwd.
thisFile = mfilename('fullpath');
root = fileparts(fileparts(fileparts(fileparts(thisFile))));
end
