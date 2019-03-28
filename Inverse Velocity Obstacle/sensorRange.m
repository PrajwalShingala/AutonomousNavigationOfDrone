function [res] = sensorRange(a,v,b)
	a = a + 0.001;
	v = v + 0.001;
	b = b + 0.001;
	m = -v(1)/v(2);
	c = a(2) - m*a(1);
	sig = b(2) - m*b(1) - c;
	sig = sig/abs(sig);
	d = a+v;
	ref = d(2) - m*d(1) - c;
	ref = ref/abs(ref);
	res = (sum((a-b).^2) < 72 && sig == ref);
end
