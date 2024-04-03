classdef FrenetPath
    properties
        t = []
        d = []
        d_d = []
        d_dd = []
        d_ddd = []
        s = []
        s_d = []
        s_dd = []
        s_ddd = []
        c_lat = 0.0
        c_lon = 0.0
        c_tot = 0.0
        x = []
        y = []
        yaw = []
        ds = []
        kappa = []
    end
    
    methods
        % Constructor
        function obj = FrenetPath()
            % This class automatically initializes its properties with default values.
        end
    end
end