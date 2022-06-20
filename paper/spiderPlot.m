% Initialize data points
% Test case 1
D1 = [69.7 100 87.0312 -6.141];  % Ours
E1 = [11.906300852909775, 0.00000000001, 0.173442, 0.23640];
D2 = [73.099 100 92.36480 -13.6989];  % Legible
E2 = [14.613, 0.00000000001, 0.00000000001, 0.00000000001];
D3 = [71.7 100 84.0788 0.0];  % Nominal
E3 = [14.9084, 0.00000000001, 0.00000000001, 0.00000000001];
D4 = [71.8, 100, 89.6834, -6.9108];  % Speed-Adjusted
E4 = [14.8209, 0.00000000001, 5.0704, 6.9];
D5 = [70.8 100 79.1859 -37.3079];  % Distant+Visible
E5 = [9.994, 0.00000000001, 0.6435, 0.72730];


% 	D1 = inputs(1,1:end);
% 	D2 = inputs(3,1:end);
% 	D3 = inputs(5,1:end);
% 	D4 = inputs(7,1:end);
% 	D5 = inputs(9,1:end);
% 	
% 	E1 = inputs(2,1:end);
% 	E2 = inputs(4,1:end);
% 	E3 = inputs(6,1:end);
% 	E4 = inputs(8,1:end);
% 	E5 = inputs(10,1:end);

	P = [D1; D2; D3; D4; D5];
	E = [E1; E2; E3; E4; E5];

	mins = [];
	maxs = [];
	for i = 1:size(D1, 2)
	    mn = min([D1(i), D2(i), D3(i), D4(i), D5(i)] - [E1(i), E2(i), E3(i), E4(i), E5(i)]);
	    mn = round(mn - 0.5);
	    
	    mx = max([D1(i), D2(i), D3(i), D4(i), D5(i)] + [E1(i), E2(i), E3(i), E4(i), E5(i)]);
	    mx = round(mx + 0.5);
	    
	    if i < 4
	        mn = max([mn, 0]);
	        mx = min([mx, 100]);
	    else
	        mx = min([mx, 0]);
	    end
	    
	    mins = [mins, mn];
	    maxs = [maxs, mx];
	end

	t = tiledlayout(1, 4);

	% Spider plot
	nexttile;
	spider_plot([D1; D3], [E1; E3], {'CoMOTO', 'Nominal'}, 3, 'Marker', 'none',...
	    'AxesLimits', [mins; maxs],...
	    'AxesLabels', {'Dst.', 'Vis.', 'Leg.', 'Nom.'},...
		'AxesFontSize', 8,...
	    'FillOption', 'on',...
	    'FillTransparency', 0.05);

	nexttile;
	spider_plot([D1; D4], [E1; E4], {'CoMOTO', 'Speed-Adj'}, 4, 'Marker', 'none',...
	    'AxesLimits', [mins; maxs],...
	    'AxesLabels', {'Dst.', 'Vis.', 'Leg.', 'Nom.'},...
		'AxesFontSize', 8,...
	    'FillOption', 'on',...
	    'FillTransparency', 0.05);

	nexttile;
	spider_plot([D1; D2], [E1; E2], {'CoMOTO', 'Legible'}, 2, 'Marker', 'none',...
	    'AxesLimits', [mins; maxs],...
	    'AxesLabels', {'Dst.', 'Vis.', 'Leg.', 'Nom.'},...
		'AxesFontSize', 8,...
	    'FillOption', 'on',...
	    'FillTransparency', 0.05);

	nexttile;
	spider_plot([D1; D5], [E1; E5], {'CoMOTO', 'Dist+Vis'}, 5, 'Marker', 'none',...
	    'AxesLimits', [mins; maxs],...
	    'AxesLabels', {'Dst.', 'Vis.', 'Leg.', 'Nom.'},...
		'AxesFontSize', 8,...
	    'FillOption', 'on',...
	    'FillTransparency', 0.05);

	t.TileSpacing = 'compact';
	t.Padding = 'compact';
	% title(t, 'Human and robot reaching for different goals')

	set(gcf, 'position', [10, 10, 1600, 350]);

	% Legend settings
	%legend('Ours', 'Legibility', 'Nominal Trajectory', 'Speed Control', 'Dist + Vis', 'Location', 'southoutside');
	pause;
