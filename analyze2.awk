BEGIN {
	get = 0;
	i = 0;
}

function dump() {
		for (x = 10; x <= 27; x++) {
			if (x in arr) {
				switch (x) {
					case 10: r = "CLEARCHAR" 
						break
					case 11: r = "AUTHSEEDG" 
						break
					case 12: r = "AUTHENTRY" 
						break
					case 13: r = "SVNSEGSET" 
						break
					case 14: r = "AUTHSEEDR" 
						break
					case 15: r = "AUTHENACK" 
						break
					case 16: r = "SVNSEGACK" 
						break
					case 17: r = "GETVENDIN" 
						break
					case 18: r = "GETVENDFU" 
						break
					case 19: r = "GETDISCIN" 
						break
					case 20: r = "UNCHARVRS" 
						break
					case 21: r = "GETDEVIIN" 
						break
					case 22: r = "UNATTCDIN" 
						break
					case 23: r = "VCARDEPRE" 
						break
					case 24: r = "HEARTBEAT" 
						break
					case 25: r = "CONNNOTIF" 
						break
					case 26: r = "DISCNOTIF" 
						break
					case 27: r = "BLEMODESW" 
						break
				}
				printf "%s: ", r
				for (y = 0; y < 100; y++) {
					if (y in arr[x]) {
						printf "%d, ", arr[x][y]
					}
				}
				printf "\n";
			}
		}
		printf "\n";
}

{ 
	gsub("^\\[0*", "", $1);
	gsub("\\]\|\<\|\>", ""); 

	if($5 == 25 && get == 0) 
	{
		get = 1; 
		i = 0; 
	}
	if ($5 == 25 && get == 1 && i > 0) 
	{
		get = 0; 
		dump();
	}
	if (get == 1) 
	{ 
		arr[$5][i++] = $1; 
		linearr[$5, i] = NR;
	} 
} 

END {
	dump();
}
