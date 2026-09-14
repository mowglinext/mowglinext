package updater

import (
	"encoding/json"
	"testing"
)

func publicationFixture() (Deployment, CustomImage) {
	d := fixture()
	d.ImageContract = 1
	d.ID = "deployment-aaaaaaaaaaaa-42-1"
	d.ReleaseTag = "v1.3.0"
	d.Source.Track = "stable"
	d.Source.Branch = "main"
	i := d.Images["mowglinext-gui"]
	p := i.Platforms["linux/arm64"]
	p.Version = d.ReleaseTag
	i.Platforms["linux/arm64"] = p
	d.Images["mowglinext-gui"] = i
	return d, CustomImage{Repository: d.Source.Repository, ReleaseTag: d.ReleaseTag, DeploymentID: d.ID, Family: "mowglinext-gui", FirmwareProtocol: 6, Revision: d.Revision, Version: d.ReleaseTag, Reference: i.Repository + "@" + p.Manifest, ImageID: p.Config}
}

func TestCustomImageMustMatchPublishedVersionAndBytes(t *testing.T) {
	d, image := publicationFixture()
	if err := validateCustomPublication(image, "linux/arm64", d, false); err != nil {
		t.Fatal(err)
	}
	for _, test := range []struct {
		name   string
		change func(*Deployment, *CustomImage)
	}{
		{"pre-updater release", func(d *Deployment, _ *CustomImage) { d.ImageContract = 0 }},
		{"relabelled image", func(_ *Deployment, i *CustomImage) {
			i.Reference = "ghcr.io/example/fake@sha256:ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff"
		}},
		{"wrong config", func(_ *Deployment, i *CustomImage) {
			i.ImageID = "sha256:ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff"
		}},
		{"fake version label", func(_ *Deployment, i *CustomImage) { i.Version = "v1.4.0" }},
		{"missing revision", func(_ *Deployment, i *CustomImage) { i.Revision = "" }},
		{"wrong release", func(_ *Deployment, i *CustomImage) { i.ReleaseTag = "v1.2.0" }},
		{"wrong family", func(_ *Deployment, i *CustomImage) { i.Family = "gps" }},
		{"unversioned production", func(d *Deployment, i *CustomImage) {
			d.ReleaseTag = "latest"
			i.ReleaseTag = "latest"
			i.Version = "latest"
		}},
		{"wrong protocol", func(_ *Deployment, i *CustomImage) { i.FirmwareProtocol = 5 }},
	} {
		t.Run(test.name, func(t *testing.T) {
			data, _ := json.Marshal(d)
			var copy Deployment
			_ = json.Unmarshal(data, &copy)
			other := image
			test.change(&copy, &other)
			if validateCustomPublication(other, "linux/arm64", copy, false) == nil {
				t.Fatal("accepted invalid publication")
			}
		})
	}
	if validateCustomPublication(image, "linux/amd64", d, false) == nil {
		t.Fatal("accepted wrong platform image")
	}
	if validateCustomPublication(image, "linux/arm64", d, true) == nil {
		t.Fatal("production prerelease accepted")
	}
}

func TestCustomBranchUsesImmutableNumberedSnapshot(t *testing.T) {
	d, image := publicationFixture()
	d.Source.Track = "custom"
	d.Source.Branch = "feat/gui-dashboard-improvements"
	d.ReleaseTag = d.ID
	image.ReleaseTag = d.ID
	image.Version = d.ID
	i := d.Images[image.Family]
	p := i.Platforms["linux/arm64"]
	p.Version = d.ID
	i.Platforms["linux/arm64"] = p
	d.Images[image.Family] = i
	if err := validateCustomPublication(image, "linux/arm64", d, true); err != nil {
		t.Fatal(err)
	}
	if validateCustomPublication(image, "linux/arm64", d, false) == nil {
		t.Fatal("branch published as production")
	}
}
