% Generate desired figures for the ACC 2020 paper using data from a CSV file

function data = csv2figs(fn, data)
	opts.width    = 16;
	opts.height   = 20;
	opts.fontType = 'Times';
	opts.fontSize = 9;
	opts.linewidth = 1;
	opts.nameprefix = char(extractBetween(fn, 1, strlength(fn)-4));
	opts.fileType = '-dpng';
	opts.res = '-r1200';

	if ~exist('data', 'var')
		data = readmatrix(fn);
	end

	t    = data(1, :);
	pn   = data(2, :);
	pe   = data(3, :);
	h    = data(4, :);
	u    = data(5, :);
	v    = data(6, :);
	w    = data(7, :);
	pn_d = data(8, :);
	pe_d = data(9, :);
	h_d  = data(10, :);
	u_d  = data(11, :);
	v_d  = data(12, :);
	w_d  = data(13, :);


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%% Plot 1: position in 3 subplots over time
	fig = figure(1);
	clf;
	subplot(3, 1, 1);
	hold on;
    grid on;
	plot(t, pn, 'LineWidth', opts.linewidth);
	plot(t, pn_d, 'LineWidth', opts.linewidth);
	xlabel("t");
	ylabel("north (m)");
	legend('actual', 'desired');
	legend('Location', 'southeast')
	axis tight;
	hold off;

	subplot(3, 1, 2);
	hold on;
    grid on;
	plot(t, pe, 'LineWidth', opts.linewidth);
	plot(t, pe_d, 'LineWidth', opts.linewidth);
	xlabel("t");
	ylabel("east (m)");
	legend('actual', 'desired');
		legend('Location', 'southeast')
    axis tight;
	hold off;

	subplot(3, 1, 3);
	hold on;
    grid on;
	plot(t, h, 'LineWidth', opts.linewidth);
	plot(t, h_d, 'LineWidth', opts.linewidth);
	xlabel("t");
	ylabel("height (m)");
	legend('actual', 'desired');
	legend('Location', 'southeast')
	axis tight;
	hold off;

% 	sgtitle("Position");
	fig.Units = 'centimeters';
	fig.Position(3) = 14;
	fig.Position(4) = 20;
	
	fig.PaperPositionMode   = 'auto';
	%saveas(fig, [opts.nameprefix '_1_pos'], opts.fileType)
    print([opts.nameprefix '_1_pos'], opts.fileType, opts.res)
    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%% Plot 2: 3d plot of desired and actual position
	fig = figure(2);
	clf;
	hold on;
    grid on;
% 	sgtitle("3D Position");
	plot3(pn, -pe, h);
	plot3(pn_d, -pe_d, h_d);
	legend('actual', 'desired');
    legend('location', 'northoutside')
	axis('tight')
    axis('equal')
	hold off;
	fig.Units = 'centimeters';
	fig.Position(3) = 10;
	fig.Position(4) = 8;

	fig.PaperPositionMode   = 'auto';
    view(20, 45);
	%saveas(fig, [opts.nameprefix '_2_pos3d'], opts.fileType)
    print([opts.nameprefix '_2_pos3d'], opts.fileType, opts.res)

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%% Plot 3: Position error plot

	p_err = sqrt((pn_d - pn).^2 + (pe_d - pe).^2 + (h_d - h).^2);

	fig = figure(3);
	clf;
	hold on;
    grid on;
% 	sgtitle("Position Error");
	plot(t, p_err);
	xlabel("t");
	ylabel("error (m)");
	axis tight;
	hold off;
	fig.Units = 'centimeters';
	fig.Position(3) = 10;
	fig.Position(4) = 8;

	fig.PaperPositionMode   = 'auto';
	%saveas(fig, [opts.nameprefix '_3_p-err'], opts.fileType)
    print([opts.nameprefix '_3_p-err'], opts.fileType, opts.res)

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	%% Plot 4: velocity error plot
	v_err = sqrt((u_d - u).^2 + (v_d - v).^2 + (w_d - w).^2);
	fig = figure(4);
	clf;
	hold on;
    grid on;
% 	sgtitle("Velocity Error");
	plot(t, v_err);
	xlabel("t");
	ylabel("error (m/s)");
    axis tight;
	hold off;
	fig.Units = 'centimeters';
	fig.Position(3) = 10;
	fig.Position(4) = 8;
	fig.PaperPositionMode   = 'auto';
	%saveas(fig, [opts.nameprefix '_4_v-err'], opts.fileType)
    print([opts.nameprefix '_4_v-err'], opts.fileType, opts.res)

end
