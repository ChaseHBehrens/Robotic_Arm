function debug(func)
    persistent debug_enabled;
    if isa(func, 'logical')
        debug_enabled = func;
        return;
    end
    if debug_enabled
        func();
    end
end