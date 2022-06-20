comoto_means = [0.526, 0.7424, 0.5992, 9.792];
comoto_stds = [0.4737, 0.0301, 0.00864, 1.6414];

maxs = [1,1,1,15];
mins = [0,0,0,0];

jl_means = [0.831, 0.533, 0.2571, 5.7943];
jl_stds = [0.1727, 0.0108, 0.00598, 0.6670];

spider_plot([comoto_means; jl_means], [comoto_stds; jl_stds], {'CoMOTO', 'Ours'}, 3, 'Marker', 'none',...
	    'AxesLimits', [mins; maxs],...
	    'AxesLabels', {'Dst.', 'Vis.', 'Leg.', 'Nom.'},...
		'AxesFontSize', 8,...
	    'FillOption', 'on',...
	    'FillTransparency', 0.05,...
        'LineWidth', 3);