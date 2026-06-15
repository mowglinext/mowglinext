package api

import "testing"

func TestParseSourcetable(t *testing.T) {
	raw := "SOURCETABLE 200 OK\r\n" +
		"Content-Type: text/plain\r\n\r\n" +
		"CAS;crtk.net;2101;Centipede;;0;FRA;43.6;1.4;0.0.0.0;0;;\r\n" +
		"NET;CENTIPEDE;Centipede;B;N;http://centipede.fr;;;\r\n" +
		"STR;TLSE;Toulouse;RTCM 3.2;1004(1),1012(1);2;GPS+GLO;CENTIPEDE;FRA;43.60;1.44;0;0;sNTRIP;none;B;N;5000;\r\n" +
		"STR;PARI;Paris;RTCM 3.2;;2;GPS+GLO+GAL;CENTIPEDE;FRA;48.85;2.35;0;0;mod;none;B;N;9600;\r\n" +
		"STR;BADLAT;Bad;RTCM 3.2;;2;GPS;NET;FRA;notanumber;1.0;0;0;;;;;;\r\n" +
		"STR;TOOFEW;x\r\n" +
		"ENDSOURCETABLE\r\n"

	stations := parseSourcetable(raw)
	if len(stations) != 2 {
		t.Fatalf("expected 2 valid STR entries, got %d", len(stations))
	}
	if stations[0].Mountpoint != "TLSE" || stations[0].Country != "FRA" {
		t.Errorf("station0 wrong: %+v", stations[0])
	}
	if stations[0].Lat != 43.60 || stations[0].Lon != 1.44 {
		t.Errorf("station0 coords wrong: lat=%v lon=%v", stations[0].Lat, stations[0].Lon)
	}
	if stations[1].Mountpoint != "PARI" || stations[1].NavSystem != "GPS+GLO+GAL" {
		t.Errorf("station1 wrong: %+v", stations[1])
	}
}

func TestParseSourcetableEmpty(t *testing.T) {
	if got := parseSourcetable("SOURCETABLE 200 OK\r\nENDSOURCETABLE\r\n"); len(got) != 0 {
		t.Fatalf("expected 0 stations, got %d", len(got))
	}
}
