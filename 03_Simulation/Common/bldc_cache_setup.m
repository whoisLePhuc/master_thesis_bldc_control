function bldc_cache_setup()
%BLDC_CACHE_SETUP  Put the Simulink cache and code-generation folders in <repo>/.cache.
%   Called by every controller init script, so that slprj/ folders and *.slxc files are
%   not created next to the models or in the current folder. The folder is git-ignored.
%   Does nothing on parallel workers (parsim uses its own temporary folders) or when the
%   cache folder is already set.
%
%   See also Simulink.fileGenControl.

if ~isempty(getCurrentTask()), return; end
cacheDir = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), '.cache');
cfg = Simulink.fileGenControl('getConfig');
if strcmp(cfg.CacheFolder, cacheDir), return; end
Simulink.fileGenControl('set', 'CacheFolder', cacheDir, 'CodeGenFolder', cacheDir, ...
                        'createDir', true);
end
