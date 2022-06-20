human_traj = [[0.035700000000000065, 0.38241, 0.96046],
[0.03180000000000005, 0.37171999999999994, 0.95841],
[0.028000000000000025, 0.36122999999999994, 0.95643],
[0.02429999999999999, 0.35095, 0.95449],
[0.02069999999999994, 0.3409, 0.95257],
[0.017099999999999893, 0.33106, 0.95068],
[0.013700000000000045, 0.32145, 0.94882],
[0.010299999999999976, 0.31203000000000003, 0.9469799999999999],
[0.006999999999999895, 0.30282, 0.94517],
[0.0037000000000000366, 0.29379, 0.94338],
[0.0005999999999999339, 0.28493, 0.94162],
[-0.0024999999999999467, 0.27625, 0.9398799999999999],
[-0.00550000000000006, 0.26771, 0.93816],
[-0.008499999999999952, 0.25930999999999993, 0.93647],
[-0.011400000000000077, 0.25103, 0.9348],
[-0.011400000000000077, 0.25103, 0.9348],
[-0.011400000000000077, 0.25103, 0.9348],
[-0.011400000000000077, 0.25103, 0.9348],
[-0.011400000000000077, 0.25103, 0.9348],
[-0.011400000000000077, 0.25103, 0.9348]];

jl_traj = [[-0.2869526426654496, 0.23432274059798877, 0.4776830083058917],
[-0.15528794353345154, 0.1410338045753715, 0.4757288233246796],
[-0.13481725750452472, 0.14608961572249418, 0.45834939601720953],
[-0.11709818097024216, 0.18399680698395662, 0.46864520876055615],
[-0.044077932868236, 0.23527458561293038, 0.4526102370603956],
[0.012399120297805422, 0.27539434751772873, 0.49424089948778027],
[0.1215648680949849, 0.2801333902133894, 0.5129190634793924],
[0.203656815167804, 0.2701649024258003, 0.5738396749316964],
[0.3077589794183647, 0.21127065983989507, 0.5995342147626082],
[0.3877334991354369, 0.16517426464989612, 0.6357415028695729],
[0.4779546345198342, 0.09696443401970364, 0.6025989743572195],
[0.5489672322197805, 0.05752339129782711, 0.5781848527444136],
[0.6144314489842243, 0.0175274780716308, 0.4953065152476301],
[0.6581985488670279, -0.01707719308292127, 0.4320461804282423],
[0.6929729854177812, -0.0494171314160111, 0.32904677157760087],
[0.7137466931464093, -0.08159555287186637, 0.262126325785101],
[0.725891222078589, -0.11518060852786528, 0.17335625820356187],
[0.7339047726079244, -0.13610826016004676, 0.11777437828564077],
[0.7395058065602982, -0.15499903393093936, 0.07976638521340648],
[0.7459735810820746, -0.1668057941081088, 0.060215946612424236],
[0.7490976331548773, -0.1630187196826684, 0.03909369384590123]];

comoto_traj = [[-0.2869526426654496, 0.23432274059798877, 0.4776830083058917],
[-0.4288440156432848, 0.17620655210775268, 0.454037350209926],
[-0.4680405341078725, 0.24076165132731336, 0.5770713079195081],
[-0.39982079618886884, 0.33986286262500875, 0.6850891319862827],
[-0.3085339860661684, 0.3759528368041872, 0.7813130603459484],
[-0.2599617790173898, 0.36439968105243925, 0.8647231102697069],
[-0.16195004763488197, 0.36928804649984764, 0.9332652038070505],
[0.0013508285302337164, 0.3534879014097084, 1.012877658505676],
[0.17237680435170918, 0.29644725200163907, 1.071612502201041],
[0.3283026239325932, 0.20369333059068168, 1.0917823122622152],
[0.4987624864163889, 0.056592384611944935, 1.0544090075762476],
[0.6857395026125946, -0.11992236261684656, 0.8829618834055435],
[0.7856739958693468, -0.24381013514352914, 0.6702451140369985],
[0.8090778859984736, -0.309895873689589, 0.551256699629714],
[0.8083503581147581, -0.35222544730903854, 0.49321797520282734],
[0.8087197018662071, -0.37083426243347645, 0.43773044413420326],
[0.8020443771106934, -0.38966520886007855, 0.42578592016505645],
[0.7897304065706687, -0.39082743306281953, 0.42498165972163293],
[0.7929833955976008, -0.34923036621952436, 0.3424066191086632],
[0.7523006602004075, -0.16489926896546248, 0.052113014108477976]];

comoto_traj(:,3) = comoto_traj(:,3) + 0.8;
jl_traj(:,3) = jl_traj(:,3) + 0.8;

plot3(human_traj(:,1), human_traj(:,2), human_traj(:,3));
hold on;
plot3(jl_traj(:,1), jl_traj(:,2), jl_traj(:,3));
plot3(comoto_traj(:,1), comoto_traj(:,2), comoto_traj(:,3));
legend("Human", "Ours", "CoMOTO");