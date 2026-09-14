package updater

import (
	"context"
	"encoding/json"
	"errors"
	"net/http"
	"net/url"
	"regexp"
	"strings"

	"github.com/mowglinext/mowglinext/pkg/updates"
)

var snapshotVersion = regexp.MustCompile(`^deployment-[a-f0-9]{12}-[0-9]+-[0-9]+$`)
var stableVersion = regexp.MustCompile(`^v(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)$`)

// Labels locate a publication; they are not proof that an image belongs to it.
// Verify the downloaded bytes against the published component/platform digests.
func verifyPublishedCustomImage(ctx context.Context, client *http.Client, image CustomImage, platform string) error {
	if !updates.RepositoryPattern.MatchString(image.Repository) || !idPattern.MatchString(image.ReleaseTag) {
		return errors.New("invalid release source")
	}
	data, err := updates.Read(ctx, client, "https://api.github.com/repos/"+image.Repository+"/releases/tags/"+url.PathEscape(image.ReleaseTag), "")
	if err != nil {
		return errors.New("image has no accessible published MowgliNext release")
	}
	var header struct {
		Draft      bool   `json:"draft"`
		Prerelease bool   `json:"prerelease"`
		Tag        string `json:"tag_name"`
	}
	if err = json.Unmarshal(data, &header); err != nil || header.Draft || header.Tag != image.ReleaseTag {
		return errors.New("image release is unpublished or has a different identity")
	}
	data, err = updates.Read(ctx, client, assetURL(image.Repository, image.ReleaseTag, "mowgli-deployment.json"), "")
	if err != nil {
		return errors.New("release does not publish an updater deployment descriptor")
	}
	var d Deployment
	if err = json.Unmarshal(data, &d); err != nil {
		return errors.New("invalid release descriptor")
	}
	return validateCustomPublication(image, platform, d, header.Prerelease)
}

func validateCustomPublication(image CustomImage, platform string, d Deployment, prerelease bool) error {
	if err := d.Validate([]string{image.Repository}); err != nil {
		return err
	}
	if d.ImageContract != 1 || d.FirmwareProtocol != image.FirmwareProtocol || d.Source.Repository != image.Repository || d.ReleaseTag != image.ReleaseTag || d.ID != image.DeploymentID || d.Revision != image.Revision {
		return errors.New("image metadata does not match a supported published deployment")
	}
	if !snapshotVersion.MatchString(d.ID) || !strings.HasPrefix(d.ID, "deployment-"+d.Revision[:12]+"-") {
		return errors.New("deployment snapshot number does not identify its source revision")
	}
	version := d.ID
	if d.Source.Track == "stable" {
		if prerelease || !stableVersion.MatchString(d.ReleaseTag) {
			return errors.New("production image needs a numbered production release")
		}
		version = d.ReleaseTag
	} else if !prerelease || !snapshotVersion.MatchString(d.ID) {
		return errors.New("branch image needs a numbered deployment snapshot")
	}
	if image.Version != version {
		return errors.New("image version does not match its published release")
	}
	published, exists := d.Images[image.Family]
	p, ok := published.Platforms[platform]
	_, digest, hasDigest := strings.Cut(image.Reference, "@")
	if !exists || !ok || !hasDigest || (digest != published.Digest && digest != p.Manifest) || !updates.Matches(published, platform, image.ImageID, nil) || p.Revision != image.Revision || p.Version != image.Version {
		return errors.New("downloaded image is not the published component for this platform")
	}
	return nil
}
